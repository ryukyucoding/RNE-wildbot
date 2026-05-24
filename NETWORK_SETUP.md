# Network & Foxglove 設定記錄

## 網路架構

```
本機電腦 (yuuuuuu, 100.103.10.56)
  ├── SSH → PN54 via Tailscale (100.107.41.55)
  └── Tailscale exit node → PN54 外網流量

PN54 (172.20.10.2)
  └── WiFi → 手機熱點 (172.20.10.1)  ← 不穩定，盡量走 Tailscale
```

---

## Foxglove 連線

- 連線網址：`ws://100.107.41.55:8765`（Tailscale 直連，不需要 SSH tunnel）
- 比原本 rosbridge 的 9090 快很多（binary 協定，不做 base64）
- **exit node 必須關閉**，否則連不上（見下方說明）

---

## 相機延遲優化

### 問題根源
1. `compress` service 指令寫錯，根本沒有在壓縮（沒有指定 output transport）
2. rosbridge 用 JSON + base64 傳輸影像，overhead 很大
3. 相機解析度 1280×720 @ 30fps，資料量太大

### 改動

**1. compress 指令修正**（`docker-compose_camera_gemini.yml`）
```bash
# 之前（錯的）
ros2 run image_transport republish raw

# 之後（正確，有壓縮）
ros2 run image_transport republish raw compressed \
  --ros-args \
  --remap in:=camera/color/image_raw \
  --remap out:=camera/color/image_raw \
  -p out.compressed.jpeg_quality:=50
```
> 注意：ROS2 Jazzy 的 quality 參數名稱是 `out.compressed.jpeg_quality`，舊的 `out.jpeg_quality` 已 deprecated 且無效。

**2. rosbridge → foxglove_bridge**（`docker-compose_foxglove_bridge.yml`）
- foxglove_bridge 用 binary WebSocket，不做 base64 轉換，延遲少一半以上
- Port：9090 → **8765**
- Foxglove 訂閱 `/camera/color/image_raw/compressed`（不是 raw）

**3. 相機解析度調整**（`docker-compose_camera_gemini.yml`）
- `1280×720 @ 30fps` → `640×480 @ 15fps`
- USB 2.0 的限制：1280×720 只支援 30fps，15fps 只能選 640×480
- 640×480 對 YOLO 等影像辨識夠用

> ⚠️ 解析度和 fps 影響所有訂閱者（包含影像辨識），JPEG quality 只影響 compressed topic（Foxglove 用）

---

## Docker DNS 修正

手機熱點的 DNS 在 Docker build 時無法使用，需在 `/etc/docker/daemon.json` 加入 IPv4 DNS：

```json
{
  "dns": ["8.8.8.8", "8.8.4.4", "2001:4860:4860::8888", "2001:4860:4860::8844"]
}
```

修改後重啟 Docker：
```bash
sudo systemctl restart docker
```

Build 時若還是 DNS 問題，在 docker-compose 的 `build:` 加上：
```yaml
build:
  network: host
```

---

## PN54 外網（GitHub push 等）

手機熱點不穩定時，用 Tailscale exit node 讓 PN54 透過本機電腦上網：

**本機電腦（一次設定）：**
```bash
tailscale up --advertise-exit-node --accept-routes
```
然後到 [admin.tailscale.com](https://admin.tailscale.com) → Machines → yuuuuuu → Edit route settings → 開啟 **Use as exit node**

**PN54 上（需要外網時執行）：**
```bash
sudo tailscale up --exit-node=100.103.10.56 --accept-routes
```

**用完關掉（避免影響 PN54 正常路由）：**
```bash
sudo tailscale up --exit-node= --accept-routes
```

> ⚠️ **重要：exit node 開著會讓 Foxglove（ws://100.107.41.55:8765）斷線。**
> - 要 git push / 連外網 → 開 exit node
> - 要看 Foxglove 畫面 → 關 exit node
