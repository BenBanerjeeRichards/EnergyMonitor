package main

import (
	"archive/zip"
	"bytes"
	"context"
	"fmt"
	"io"
	"log/slog"
	"os"
	"strings"
	"time"

	"github.com/minio/minio-go/v7"
	"github.com/minio/minio-go/v7/pkg/credentials"
)

type Config struct {
	SourceAccess   string
	SourceSecret   string
	SourceEndpoint string
	SourceBucket   string
	SourceName     string
	SourceSecure   bool
	TargetAccess   string
	TargetSecret   string
	TargetEndpoint string
	TargetBucket   string
	TargetName     string
	TargetSecure   bool
	Archive        bool
}

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	slog.SetDefault(logger)

	err := sync(loadConfig())
	if err != nil {
		slog.Error(fmt.Sprintf("Sync Failed: %v", err))
		os.Exit(1)
	} else {
		slog.Info("Sync completed successfully")
	}
}

func sync(config Config) error {
	slog.Info(fmt.Sprintf("Syncing from %s to %s", config.SourceName, config.TargetName))
	sourceClient, err := minio.New(config.SourceEndpoint, &minio.Options{
		Creds:  credentials.NewStaticV4(config.SourceAccess, config.SourceSecret, ""),
		Secure: config.SourceSecure,
	})
	if err != nil {
		return err
	}

	targetClient, err := minio.New(config.TargetEndpoint, &minio.Options{
		Creds:  credentials.NewStaticV4(config.TargetAccess, config.TargetSecret, ""),
		Secure: config.TargetSecure,
	})
	if err != nil {
		return err
	}

	lastKey, err := getLastKey(*targetClient, config)
	if err != nil {
		return fmt.Errorf("failed to get last access key: %v", err)
	}

	newFinalKey, err := syncSince(config, *sourceClient, *targetClient, lastKey)
	if err != nil {
		return fmt.Errorf("failed to sync files: %v", err)
	}

	if newFinalKey != "" {
		err = putLastKey(*targetClient, config, newFinalKey)
		if err != nil {
			return fmt.Errorf("failed to write new sync key: %v", err)
		}
	}
	return nil
}

func syncSince(config Config, sourceClient minio.Client, targetClient minio.Client, since string) (string, error) {
	slog.Info(fmt.Sprintf("Running sync from bucket %s (name=%s) from key `%s`", config.SourceBucket, config.SourceName, since))
	if config.Archive {
		return syncArchive(config, sourceClient, targetClient, since)
	} else {
		return syncIndividualFiles(config, sourceClient, targetClient, since)
	}
}

func syncArchive(config Config, sourceClient minio.Client, targetClient minio.Client, since string) (string, error) {
	now := time.Now()
	dayString := now.Format("2006-01-02")
	slog.Info(fmt.Sprintf("Using archive so only archiving up to %s", dayString))
	ctx := context.Background()

	archive, err := os.CreateTemp("", "archive")
	if err != nil {
		return "", fmt.Errorf("failed to create temp archive: %v", err)
	}
	zipWriter := zip.NewWriter(archive)
	finalKey := ""

	for object := range sourceClient.ListObjects(ctx, config.SourceBucket, minio.ListObjectsOptions{StartAfter: since, Recursive: false}) {
		if object.Key > dayString {
			continue
		}
		slog.Info(fmt.Sprintf("Downloading object %s/%s to tmp file", config.SourceBucket, object.Key))
		getObj, err := sourceClient.GetObject(ctx, config.SourceBucket, object.Key, minio.GetObjectOptions{})
		if err != nil {
			return "", fmt.Errorf("failed to download object %s: %v", object.Key, err)
		}
		slog.Info(fmt.Sprintf("Downlaoded file %s", object.Key))

		zipFileWriter, err := zipWriter.Create(object.Key)
		if err != nil {
			return "", fmt.Errorf("failed to create archive writer %s: %v", object.Key, err)
		}
		_, err = io.Copy(zipFileWriter, getObj)
		if err != nil {
			return "", fmt.Errorf("failed to write to zip %s: %v", object.Key, err)
		}
		finalKey = object.Key
	}

	err = zipWriter.Close()
	if err != nil {
		return "", fmt.Errorf("faild to close zip writer: %v", err)

	}
	backupName := fmt.Sprintf("%s.zip", dayString)
	archiveInfo, err := archive.Stat()
	if err != nil {
		return "", fmt.Errorf("failed to stat archive: %v", err)
	}
	slog.Info(fmt.Sprintf("Downloaded all objects into zip of size %db, uploading zip to target...", archiveInfo.Size()))

	_, err = archive.Seek(0, 0)
	if err != nil {
		return "", fmt.Errorf("failed to seek to start of archive: %v", err)
	}

	_, err = targetClient.PutObject(ctx, config.TargetBucket, backupName, archive, archiveInfo.Size(), minio.PutObjectOptions{})
	if err != nil {
		return "", fmt.Errorf("failed to upload archive to zip: %v", err)
	}
	return finalKey, nil
}

func syncIndividualFiles(config Config, sourceClient minio.Client, targetClient minio.Client, since string) (string, error) {
	ctx := context.Background()
	finalKey := ""
	for object := range sourceClient.ListObjects(ctx, config.SourceBucket, minio.ListObjectsOptions{StartAfter: since, Recursive: false}) {
		slog.Info(fmt.Sprintf("Syncing object with key %s", object.Key))
		obj, err := sourceClient.GetObject(ctx, config.SourceBucket, object.Key, minio.GetObjectOptions{})
		if err != nil {
			return "", fmt.Errorf("failed to get object with key %s from source bucket: %v", object.Key, err)
		}
		st, err := obj.Stat()
		if err != nil {
			return "", fmt.Errorf("failed to stat object: %v", err)
		}

		_, err = targetClient.PutObject(ctx, config.TargetBucket, object.Key, obj, st.Size, minio.PutObjectOptions{})
		if err != nil {
			return "", fmt.Errorf("failed to put object to target bucket: %v", err)
		}
		finalKey = object.Key
	}

	return finalKey, nil

}

func getLastKey(targetClient minio.Client, config Config) (string, error) {
	key := fmt.Sprintf("__LAST_SYNC_%s", config.TargetName)
	ctx := context.Background()
	obj, err := targetClient.GetObject(ctx, config.TargetBucket, key, minio.GetObjectOptions{})
	if err != nil {
		minioErr := minio.ToErrorResponse(err)
		if minioErr.Code == "NoSuchKey" {
			return "", nil
		} else {
			return "", err
		}
	}
	buf := new(strings.Builder)
	_, err = io.Copy(buf, obj)
	if err != nil {
		return "", nil // Does not exist - possible issue with google implementation of s3 api means not detected above
	}
	return buf.String(), nil
}

func putLastKey(targetClient minio.Client, config Config, lastKey string) error {
	key := fmt.Sprintf("__LAST_SYNC_%s", config.TargetName)
	ctx := context.Background()
	_, err := targetClient.PutObject(ctx, config.TargetBucket, key, bytes.NewReader([]byte(lastKey)), int64(len(lastKey)), minio.PutObjectOptions{})
	if err != nil {
		return err
	}
	return nil
}

func loadConfig() Config {
	return Config{
		SourceAccess:   getEnv("SOURCE_ACCESS"),
		SourceSecret:   getEnv("SOURCE_SECRET"),
		SourceEndpoint: getEnv("SOURCE_ENDPOINT"),
		SourceBucket:   getEnv("SOURCE_BUCKET"),
		SourceName:     getEnv("SOURCE_NAME"),
		SourceSecure:   getEnv("SOURCE_SECURE") != "false",
		TargetAccess:   getEnv("TARGET_ACCESS"),
		TargetSecret:   getEnv("TARGET_SECRET"),
		TargetEndpoint: getEnv("TARGET_ENDPOINT"),
		TargetBucket:   getEnv("TARGET_BUCKET"),
		TargetName:     getEnv("TARGET_NAME"),
		TargetSecure:   getEnv("TARGET_SECURE") != "false",
		Archive:        getEnv("ARCHIVE") == "true",
	}
}

func getEnv(name string) string {
	s, ok := os.LookupEnv(name)
	if !ok {
		panic("No env variable " + name)
	}
	return s
}
