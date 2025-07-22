#!/bin/env bash
docker build $1 -t ai-agent_spot_test:main -f ollama.Dockerfile .
