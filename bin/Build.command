#!/bin/bash

# Make directory current
cd `dirname $0`

# Update codebase
# git pull origin master

# Build project
xcodebuild -project ../xcode4/dynmx.xcodeproj

# Copy executable
cp ../xcode4/build/Release/dynmx.app/Contents/MacOS/dynmx .


