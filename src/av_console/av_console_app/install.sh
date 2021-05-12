#!/bin/sh

pwd=`pwd`
exe=${pwd}/app/av_console
icon=${pwd}/icon/icon.jpg
newline="\n"

output="av_console.desktop"

if [ -f "$output" ]; then
	rm "$output"
fi

echo "[Desktop Entry]" >> "$output"
echo "Type=Application" >> "$output"
echo "Exec=bash -i -c ${exe}" >> "$output"
echo "Name=av_console" >> "$output"
echo "GenericName=AV_CONSOLE" >> "$output"
echo "Icon=${icon}" >> "$output"
echo "Terminal=false" >> "$output"
echo "Categories=Development" >> "$output"

chmod a+x "$output"

if [ -f "~/Desktop/$output" ]; then
	rm "~/Desktop/$output"
fi

cp "$pwd/$output" ~/Desktop/$output

echo "generate desktop shortcut ok."
