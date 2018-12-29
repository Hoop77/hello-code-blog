#!/bin/bash
rm -r docs
hugo
echo -n "hellocodeblog.com" > docs/CNAME