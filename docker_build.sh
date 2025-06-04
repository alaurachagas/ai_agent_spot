#!/bin/env bash
docker build $1 -t ai_agent-spot:main -f Dockerfile .
