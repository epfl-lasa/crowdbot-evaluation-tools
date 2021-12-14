#!/bin/bash
# sudo apt install imagemagick
# 1. while-loop framework: https://unix.stackexchange.com/a/121297
# 2. 'no images defined' error : https://askubuntu.com/a/1085339
# 3. high-quality conversion: https://stackoverflow.com/a/13784772/7961693
# convert -density 300 -trim test.pdf -quality 100 test.jpg
find . -type f -name '*.pdf' -print0 |
    while IFS= read -r -d '' file; do
        convert -verbose -density 300 -quality 100 "${file}" "${file%.*}.png"
    done
