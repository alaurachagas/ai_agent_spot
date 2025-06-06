#!/bin/env bash
docker build $1 -t ai-agent_spot:main -f Dockerfile .
