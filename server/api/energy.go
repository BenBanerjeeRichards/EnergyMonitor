package main

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"log/slog"
	"net/http"
	"time"

	"github.com/google/uuid"
	"github.com/minio/minio-go/v7"
	amqp "github.com/rabbitmq/amqp091-go"
)

type energySync struct {
	minioClient *minio.Client
	rmqChannel  *amqp.Channel
	bucketName  string
	queueName   string
}

func (e *energySync) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		slog.Error(fmt.Sprintf("failed to decode request: %v", err))
		w.WriteHeader(400)
		return
	}

	ctx := context.Background()
	objectName := generateFileName()
	_, err = e.minioClient.PutObject(ctx, e.bucketName, objectName, bytes.NewReader(requestBytes), int64(len(requestBytes)), minio.PutObjectOptions{})
	if err != nil {
		slog.Error(fmt.Sprintf("failed to put file to minio bucket %s: %v", e.bucketName, err))
		w.WriteHeader(500)
		return
	}

	err = e.rmqChannel.Publish("", e.queueName, true, false, amqp.Publishing{
		Body: []byte(objectName),
	})
	if err != nil {
		slog.Error(fmt.Sprintf("failed to publish message to energy-sync queue: %v", err))
		w.WriteHeader(500)
		return
	}

	slog.Info(fmt.Sprintf("Successfully handled message and saved to %s", objectName))
}

func generateFileName() string {
	u := uuid.New().String()
	now := time.Now().Format(time.RFC3339)
	return fmt.Sprintf("%s-%s.txt", now, u)
}
