#! /bin/bash

INTERFACE="$1"

if [ -z "$INTERFACE" ]; then
    echo "Usage: $0 <defaultinterface>"
    exit 1
fi

sudo iptables -t nat -A POSTROUTING -o "$INTERFACE" -j MASQUERADE
sudo iptables -A FORWARD -i zeth -o "$INTERFACE" -j ACCEPT
sudo iptables -A FORWARD -i "$INTERFACE" -o zeth -m state --state RELATED,ESTABLISHED -j ACCEPT
