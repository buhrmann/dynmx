#!/bin/bash

# Make directory current
cd `dirname $0`

# Update codebase
git pull origin master

# Build project
xcodebuild -project ../xcode4/dynmx.xcodeproj CONFIGURATION_BUIL_DIR="../xcode4" CONFIGURATION_TEMP_DIR="../xcode4/build/dynmx.build/Release"





