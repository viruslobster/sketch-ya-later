#!/bin/bash
for filename in pickleJar/*.pts; do
    python etch.py $filename
done
