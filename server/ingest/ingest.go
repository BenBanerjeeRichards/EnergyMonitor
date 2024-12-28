package main

import (
	"context"
	"errors"
	"fmt"
	"io"
	"log/slog"
	"os"
	"slices"
	"strconv"
	"strings"
	"time"

	influxdb2 "github.com/influxdata/influxdb-client-go"
	"github.com/influxdata/influxdb-client-go/api/write"
	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
	amqp "github.com/rabbitmq/amqp091-go"
)

const (
	SensorErratic      = iota
	SensorDisconnected = iota
	SensorOk           = iota
)

type SyncData struct {
	timestamp   int64
	sensorId    string
	version     string
	rssi        int // WiFi strength
	intervals   []int
	sensorState int
	firstUpload bool
}

type Config struct {
	influxEndpoint           string
	influxToken              string
	influxOrg                string
	influxBucket             string
	minioEndpoint            string
	minioAccess              string
	minioSecret              string
	minioBucket              string
	minioSecure              bool
	impulsesPerKwh           int64
	rabbitmqConnectionString string
}

func failOnError(err error, msg string) {
	if err != nil {
		slog.Error(fmt.Sprintf("Error: %s:  %v", msg, err))
		panic("error")
	}
}

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	slog.SetDefault(logger)
	config := loadConfig()

	minioClient, err := minio.New(config.minioEndpoint, &minio.Options{
		Creds:  credentials.NewStaticV4(config.minioAccess, config.minioSecret, ""),
		Secure: config.minioSecure,
	})
	failOnError(err, "Failed to connect to minio")

	mqConn, err := amqp.Dial(config.rabbitmqConnectionString)
	failOnError(err, "Failed to connect")
	ch, err := mqConn.Channel()
	failOnError(err, "Failed to open a channel")
	defer ch.Close()

	q := configureQueue(ch)

	msgs, err := ch.Consume(
		q.Name, // queue
		"",     // consumer
		false,  // auto-ack
		false,  // exclusive
		false,  // no-local
		false,  // no-wait
		nil,    // args
	)
	failOnError(err, "Failed to register a consumer")

	var forever chan struct{}

	go func() {
		for d := range msgs {
			slog.Info(fmt.Sprintf("Received a message: %s", d.Body))
			err := ingestFile(*minioClient, config, string(d.Body))
			if err != nil {
				slog.Error(fmt.Sprintf("Failed to process message: %s: %v", d.Body, err))
				d.Nack(false, false) // Don't requeue as we have a DLQ
			} else {
				slog.Info(fmt.Sprintf("Successfully processed message: %s", d.Body))
				d.Ack(false)
			}
		}
	}()

	slog.Info(fmt.Sprintf("Waiting for messages to queue %s", q.Name))
	<-forever

	failOnError(err, "Failed to declare a queue")
}

func configureQueue(ch *amqp.Channel) amqp.Queue {
	err := ch.ExchangeDeclare("dlq", "direct", true, false, false, false, nil)
	failOnError(err, "Failed to declare DLQ")

	args := amqp.Table{}
	args["x-dead-letter-exchange"] = "dlq"

	q, err := ch.QueueDeclare(
		"energy-sync", // name
		true,          // durable
		false,         // delete when unused
		false,         // exclusive
		false,         // no-wait
		args,          // arguments
	)
	failOnError(err, "Failed to declare queue")

	_, err = ch.QueueDeclare(
		"energy-sync-dlq", // name
		true,              // durable
		false,             // delete when unused
		false,             // exclusive
		false,             // no-wait
		nil,               // arguments
	)
	failOnError(err, "Failed to declare dlq")

	err = ch.QueueBind("energy-sync-dlq", "energy-sync", "dlq", false, nil)
	failOnError(err, "Failed to bind to dlq")
	return q
}

func ingestFile(minioClient minio.Client, config Config, key string) error {
	ctx := context.Background()
	obj, err := minioClient.GetObject(ctx, config.minioBucket, key, minio.GetObjectOptions{})
	if err != nil {
		return err
	}
	buf := new(strings.Builder)
	_, err = io.Copy(buf, obj)
	if err != nil {
		return fmt.Errorf("failed to read minio object into string: %v", err)
	}
	contentsStr := buf.String()
	sync, err := parseSyncRequest(contentsStr)
	if err != nil {
		return fmt.Errorf("failed to parse sync: %v", err)
	}
	points := computeEnergyPoints(sync, config)
	err = syncDataPoints(config, points)
	if err != nil {
		return fmt.Errorf("failed to sync %d data points with dynamodb: %v", len(points), err)
	}
	return nil
}

func syncDataPoints(config Config, points []Point) error {
	if len(points) == 0 {
		slog.Info("Ignoring data points as it is an empty list")
		return nil
	}
	ctx := context.Background()
	influxClient := influxdb2.NewClient(config.influxEndpoint, config.influxToken)
	writeApi := influxClient.WriteAPIBlocking(config.influxOrg, config.influxBucket)
	queryApi := influxClient.QueryAPI(config.influxOrg)
	// Query to see if the records already exist
	start := points[len(points)-1].timestamp
	end := points[0].timestamp
	if start > end {
		tmp := start
		start = end
		end = tmp
	}

	if len(points) >= 2 {
		diff := float64(end - start)
		timeStart := int(float64(start) + (0.1 * diff))
		timeEnd := int(float64(end) - (0.1 * diff))
		result, err := queryApi.Query(ctx, fmt.Sprintf(`from(bucket:"%s")|> range(start: %d, stop: %d)`, config.influxBucket, timeStart, timeEnd))
		if err != nil {
			return fmt.Errorf("failed to check for existing data: %v", err)
		}
		if result.Next() {
			// Existing data
			slog.Info(fmt.Sprintf("Existing data found between range %d-%d (trimmed from %d-%d) - skipping influx sync", start, end, timeStart, timeEnd))
			return nil
		}
	} else {
		slog.Info(fmt.Sprintf("Only %d points to skipping influx duplicate check", len(points)))
	}

	influxPoints := make([]*write.Point, 0)
	for _, p := range points {
		ts := time.Unix(int64(p.timestamp), 0)
		p := influxdb2.NewPointWithMeasurement("energy").
			AddTag("unit", "W").
			AddField("energy", p.energyWatts).
			SetTime(ts)
		influxPoints = append(influxPoints, p)
	}
	err := writeApi.WritePoint(ctx, influxPoints...)
	if err != nil {
		return fmt.Errorf("failed to write %d points to influx: %v", len(influxPoints), err)
	}

	slog.Info(fmt.Sprintf("Wrote %d points to influxdb", len(points)))
	return nil
}

type Point struct {
	timestamp   int
	energyWatts int
}

func computeEnergyPoints(sync SyncData, config Config) []Point {
	points := make([]Point, 0)
	if sync.firstUpload {
		slog.Warn(fmt.Sprintf("First upload detected for sync timestamp=%d", sync.timestamp))
	}
	currentTimestampOffsetUs := 0.0
	for _, inteval := range slices.Backward(sync.intervals) {
		intervalSeconds := float64(inteval) / (1000.0 * 1000.0)
		energy := float64(3600*1000) / (intervalSeconds * float64(config.impulsesPerKwh))

		timestampSeconds := float64(sync.timestamp) - (currentTimestampOffsetUs / (1000.0 * 1000.0))
		currentTimestampOffsetUs += float64(inteval)
		points = append(points, Point{timestamp: int(timestampSeconds), energyWatts: int(energy)})
	}
	return points
}

func parseSyncRequest(requestStr string) (SyncData, error) {
	pairs := strings.Split(requestStr, ";")
	m := make(map[string]string)
	for _, pair := range pairs {
		kv := strings.Split(pair, "=")
		if len(kv) != 2 {
			return SyncData{}, errors.New("invalid request")
		}
		key := kv[0]
		value := kv[1]
		m[key] = value
	}

	syncData := SyncData{}

	timestampStr, ok := m["timestamp"]
	if ok {
		timestamp, err := strconv.Atoi(timestampStr)
		if err != nil {
			return SyncData{}, errors.New("timestamp must be an integer")
		}
		syncData.timestamp = int64(timestamp)
	} else {
		return SyncData{}, errors.New("missing timestamp")
	}

	sensorId, ok := m["sensorId"]
	if ok {
		syncData.sensorId = sensorId
	} else {
		return SyncData{}, errors.New("missing sensorId")
	}
	rssiStr, ok := m["rssi"]
	if ok {
		rssi, err := strconv.Atoi(rssiStr)
		if err != nil {
			return SyncData{}, errors.New("rssi must be an integer")
		}
		syncData.rssi = rssi
	} else {
		return SyncData{}, errors.New("missing rssi")
	}

	version, ok := m["version"]
	if ok {
		syncData.version = version
	} else {
		return SyncData{}, errors.New("missing version")
	}

	sensor, ok := m["sensor"]
	if ok {
		if sensor == "ok" {
			syncData.sensorState = SensorOk
		} else if sensor == "erratic" {
			syncData.sensorState = SensorErratic
		} else if sensor == "discon" {
			syncData.sensorState = SensorDisconnected
		}
	}

	firstUpload, ok := m["firstUpload"]
	if ok {
		syncData.firstUpload = firstUpload == "true"
	}

	intervalsStr, ok := m["values"]
	if ok {
		intervalsStrParts := strings.Split(intervalsStr, ",")
		if len(strings.TrimSpace(intervalsStr)) == 0 {
			intervalsStrParts = []string{}
		} else if strings.HasSuffix(intervalsStr, ",") {
			// Should always be the case - to keep code on esp simple
			intervalsStrParts = intervalsStrParts[:len(intervalsStrParts)-1]
		}
		for _, part := range intervalsStrParts {
			interval, err := strconv.Atoi(part)
			if err != nil {
				return SyncData{}, errors.New("interval must be an integer")
			}
			syncData.intervals = append(syncData.intervals, interval)
		}
	} else {
		return SyncData{}, errors.New("missing intervals")
	}

	return syncData, nil
}

func getEnvInt(name string) int64 {
	s := getEnv(name)
	i, err := strconv.Atoi(s)
	if err != nil {
		panic("Failed to read integer env variable " + name)
	}
	return int64(i)
}

func getEnv(name string) string {
	s, ok := os.LookupEnv(name)
	if !ok {
		panic("No env variable " + name)
	}
	return s
}

func loadConfig() Config {
	return Config{
		influxEndpoint:           getEnv("INFLUX_ENDPOINT"),
		influxToken:              getEnv("INFLUX_TOKEN"),
		influxOrg:                getEnv("INFLUX_ORG"),
		influxBucket:             getEnv("INFLUX_BUCKET"),
		minioAccess:              getEnv("MINIO_ACCESS"),
		minioSecret:              getEnv("MINIO_SECRET"),
		minioBucket:              getEnv("MINIO_BUCKET"),
		minioSecure:              getEnv("MINIO_SECURE") != "false",
		minioEndpoint:            getEnv("MINIO_ENDPOINT"),
		impulsesPerKwh:           getEnvInt("IMPL_PER_KWH"),
		rabbitmqConnectionString: getEnv("RABBITMQ_CONNECTION_STRING"),
	}
}
