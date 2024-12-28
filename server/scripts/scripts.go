package main

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"strings"

	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
)

type Config struct {
	MinioAccess   string
	MinioSecret   string
	MinioEndpoint string
	MinioBucket   string
	MinioSecure   bool
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

	err := fixNames(loadConfig())
	if err != nil {
		slog.Error(fmt.Sprintf("Sync Failed: %v", err))
	} else {
		slog.Info("OK")
	}
}

func fixNames(config Config) error {
	client, err := minio.New(config.MinioEndpoint, &minio.Options{
		Creds:  credentials.NewStaticV4(config.MinioAccess, config.MinioSecret, ""),
		Secure: config.MinioSecure,
	})
	failOnError(err, "Failed to connect to minio")
	ctx := context.Background()

	for info := range client.ListObjects(ctx, config.MinioBucket, minio.ListObjectsOptions{}) {
		if !strings.HasPrefix(info.Key, "2024") && !strings.HasPrefix(info.Key, "__") {
			slog.Info(fmt.Sprintf("Found incorrect key %s", info.Key))

			parts := strings.Split(info.Key, "-2024-")
			if len(parts) != 2 {
				panic("Expected parts to be of length 2")
			}
			p1 := strings.Replace(parts[1], ".txt", "", 1)
			correctKey := fmt.Sprintf("2024-%s-%s.txt", p1, parts[0])
			slog.Info(fmt.Sprintf("Correct key %s", correctKey))

			_, err := client.CopyObject(ctx, minio.CopyDestOptions{Bucket: config.MinioBucket, Object: correctKey},
				minio.CopySrcOptions{Bucket: config.MinioBucket, Object: info.Key})
			failOnError(err, "failed to copy")
			err = client.RemoveObject(ctx, config.MinioBucket, info.Key, minio.RemoveObjectOptions{})
			failOnError(err, "failed to remove")
		}
	}

	return nil
}

func loadConfig() Config {
	return Config{
		MinioAccess:   getEnv("MINIO_ACCESS"),
		MinioSecret:   getEnv("MINIO_SECRET"),
		MinioEndpoint: getEnv("MINIO_ENDPOINT"),
		MinioBucket:   getEnv("MINIO_BUCKET"),
		MinioSecure:   getEnv("MINIO_SECURE") != "false",
	}
}

func getEnv(name string) string {
	s, ok := os.LookupEnv(name)
	if !ok {
		panic("No env variable " + name)
	}
	return s
}
