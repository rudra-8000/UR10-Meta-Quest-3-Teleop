#!/bin/bash
# Get the laptop's local IP (the one your Quest can reach)
LAN_IP=$(hostname -I | awk '{print $1}')

openssl req -x509 -newkey rsa:2048 \
  -keyout key.pem -out cert.pem \
  -days 365 -nodes \
  -subj "/CN=${LAN_IP}" \
  -addext "subjectAltName=IP:${LAN_IP},IP:127.0.0.1,DNS:localhost"

echo ""
echo "✓ cert.pem + key.pem generated"
echo "  Your LAN IP: ${LAN_IP}"
echo "  Open on Quest: https://${LAN_IP}:8443/"
