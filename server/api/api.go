package main

import (
	"fmt"
	"log/slog"
	"net/http"
	"os"

	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	amqp "github.com/rabbitmq/amqp091-go"
	metrics "github.com/slok/go-http-metrics/metrics/prometheus"
	"github.com/slok/go-http-metrics/middleware"
	"github.com/slok/go-http-metrics/middleware/std"
)

type Config struct {
	minioEndpoint            string
	minioAccess              string
	minioSecret              string
	minioBucket              string
	minioSecure              bool
	rabbitmqConnectionString string
	apiKeyPath               string
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
	apiKeys, err := loadApiKeys(config.apiKeyPath)
	failOnError(err, "get api keys")

	mdlw := middleware.New(middleware.Config{
		Recorder: metrics.NewRecorder(metrics.Config{}),
	})

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

	mux := http.NewServeMux()

	// this is an endpoint called by the nginx auth_request (setup in the k8s nginx ingress resource)
	// it allows for api key access to items on my api. subdomain
	mux.Handle("/auth", &authenticateHandler{apiKeys: apiKeys})
	// this actually syncs energy data from then meter
	mux.Handle("/energy/sync", &energySync{
		minioClient: minioClient,
		rmqChannel:  ch,
		bucketName:  config.minioBucket,
		queueName:   "energy-sync",
	})

	// Prom middleware for metrics
	h := std.Handler("", mdlw, mux)

	slog.Info("serving metrics at: :9090")
	go http.ListenAndServe(":9090", promhttp.Handler())

	slog.Info("Listening on :8080")
	err = http.ListenAndServe(":8080", h)
	failOnError(err, "failed to start server")
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
		minioAccess:              getEnv("MINIO_ACCESS"),
		minioSecret:              getEnv("MINIO_SECRET"),
		minioBucket:              getEnv("MINIO_BUCKET"),
		minioSecure:              getEnv("MINIO_SECURE") != "false",
		minioEndpoint:            getEnv("MINIO_ENDPOINT"),
		rabbitmqConnectionString: getEnv("RABBITMQ_CONNECTION_STRING"),
		apiKeyPath:               getEnv("API_KEY_PATH"),
	}
}
