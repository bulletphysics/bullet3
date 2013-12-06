#!/bin/sh
./premake4_osx --ios xcode4
xcodebuild -project xcode4ios/AppUnitTest.xcodeproj -configuration Release -arch armv7
