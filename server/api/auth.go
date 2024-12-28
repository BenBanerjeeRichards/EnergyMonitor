package main

import (
	"fmt"
	"log/slog"
	"net/http"
	"net/url"
	"os"
	"slices"

	"gopkg.in/yaml.v2"
)

type authenticateHandler struct {
	apiKeys map[string][]string
}

func loadApiKeys(path string) (map[string][]string, error) {
	dat, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("failed to load api keys from path %s: %v", path, err)
	}
	m := make(map[string][]string)
	err = yaml.Unmarshal([]byte(dat), &m)
	if err != nil {
		return nil, fmt.Errorf("failed to parse yaml api keys: %v", err)
	}
	return m, nil
}

func (h *authenticateHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	uri := req.Header.Get("X-Original-Url")
	u, err := url.Parse(uri)
	if err != nil {
		slog.Error(fmt.Sprintf("Failed to parse uri %s: %v", uri, err))
		w.WriteHeader(500)
		return
	}
	apiKeyFromUri := u.Query().Get("apiKey")
	apiKeyFromHeader := req.Header.Get("X-Api-Key")

	if apiKeyFromUri == "" && apiKeyFromHeader == "" {
		slog.Info(fmt.Sprintf("No api key provided for uri %s", uri))
		w.WriteHeader(401)
		return
	}

	allowedApiKeys, ok := h.apiKeys[u.Path]
	if !ok {
		slog.Info(fmt.Sprintf("Path %s not found in api key config", uri))
		w.WriteHeader(401)
		return
	}

	if slices.Contains(allowedApiKeys, apiKeyFromHeader) {
		w.WriteHeader(200)
		return
	}
	if slices.Contains(allowedApiKeys, apiKeyFromUri) {
		w.WriteHeader(200)
		return
	}
	w.WriteHeader(401)
}
