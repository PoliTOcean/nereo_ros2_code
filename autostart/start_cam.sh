#!/usr/bin/env bash

gst-launch-1.0 v4l2src ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=10.0.0.69 port=5000