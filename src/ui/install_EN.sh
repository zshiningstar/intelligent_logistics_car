#!/bin/sh

pwd=`pwd`
exe=${pwd}/smartcar
icon=${pwd}/car.jpg
newline="\n"

output="smartcar.desktop"

if [ -f "$output" ]; then
	rm "$output"
fi

echo "[Desktop Entry]" >> "$output"
echo "Type=Application" >> "$output"
echo "Exec=bash -i -c ${exe}" >> "$output"
echo "Name=物流车" >> "$output"
echo "GenericName=物流车" >> "$output"
echo "Icon=${icon}" >> "$output"
echo "Terminal=false" >> "$output"
echo "Categories=Development" >> "$output"

chmod a+x "$output"

if [ -f "~/Desktop/$output" ]; then
	rm "~/Desktop/$output"
fi

cp "$pwd/$output" ~/Desktop/$output

echo "generate desktop shortcut ok."
