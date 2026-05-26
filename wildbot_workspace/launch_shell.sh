#!/bin/bash
# run.sh

IMAGE_NAME="wildbot_workspace"
DOCKERFILE="Dockerfile"
MARKER=".docker_build_hash"

# 計算目前 Dockerfile 的 hash
CURRENT_HASH=$(sha256sum $DOCKERFILE | awk '{print $1}')

# 確認是否需要重新 build
NEED_BUILD=false

if ! docker image inspect $IMAGE_NAME &>/dev/null; then
  echo "[wildbot] Image 不存在，開始 build..."
  NEED_BUILD=true
elif [ ! -f $MARKER ]; then
  echo "[wildbot] 找不到 build 記錄，重新 build..."
  NEED_BUILD=true
elif [ "$CURRENT_HASH" != "$(cat $MARKER)" ]; then
  echo "[wildbot] Dockerfile 有變更，重新 build..."
  NEED_BUILD=true
fi

if [ "$NEED_BUILD" = true ]; then
  docker build --network=host -t $IMAGE_NAME . || { echo "[wildbot] Build 失敗"; exit 1; }
  echo $CURRENT_HASH > $MARKER
  echo "[wildbot] Build 完成"
fi

NETWORK_NAME="compose_my_bridge_network"
COMPOSE_PROJECT="compose"
COMPOSE_NETWORK="my_bridge_network"

# 確保 network 存在（加上 compose 預期的 labels）
if ! docker network inspect $NETWORK_NAME &>/dev/null; then
  echo "[wildbot] Network 不存在，建立 $NETWORK_NAME..."
  docker network create \
    --driver bridge \
    --label com.docker.compose.network=$COMPOSE_NETWORK \
    --label com.docker.compose.project=$COMPOSE_PROJECT \
    --label com.docker.compose.version="$(docker compose version --short 2>/dev/null || echo 2.0.0)" \
    $NETWORK_NAME
fi

# 啟動
echo "[wildbot] starting container..."
if docker ps -a --format '{{.Names}}' | grep -qx wildbot; then
  echo "[wildbot] 移除舊 wildbot container（須用新 build 的 image 才會生效）..."
  docker rm -f wildbot
fi
docker run -it \
  --name wildbot \
  --rm \
  --network $NETWORK_NAME \
  --env-file ./docker/compose/.env \
  -v $(pwd)/workspaces:/workspaces \
  -v $(pwd)/docker/compose/configs:/configs \
  $IMAGE_NAME \
  bash

# 清理 network（如果沒有其他 container 在用）
if docker network inspect $NETWORK_NAME &>/dev/null; then
  CONNECTED=$(docker network inspect $NETWORK_NAME --format '{{len .Containers}}')
  if [ "$CONNECTED" -eq 0 ]; then
    echo "[wildbot] 沒有其他 container 使用 $NETWORK_NAME，移除..."
    docker network rm $NETWORK_NAME
  else
    echo "[wildbot] $NETWORK_NAME 仍有 $CONNECTED 個 container 在使用，保留"
  fi
fi