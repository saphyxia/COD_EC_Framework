#!/bin/sh

find ../ -name "*.bak" \
    -or -name "*.ddk" \
    -or -name "*.edk" \
    -or -name "*.lst" \
    -or -name "*.lnp" \
    -or -name "*.mpf" \
    -or -name "*.mpj" \
    -or -name "*.obj" \
    -or -name "*.omf" \
    -or -name "*.plg" \
    -or -name "*.rpt" \
    -or -name "*.tmp" \
    -or -name "*.__i" \
    -or -name "*.crf" \
    -or -name "*.o" \
    -or -name "*.d" \
    -or -name "*.axf" \
    -or -name "*.tra" \
    -or -name "*.dep" \
    -or -name "JLinkLog.txt" \
    -or -name "*.iex" \
    -or -name "*.htm" \
    -or -name "*.sct" \
    -or -name "*.map" \
    -or -name "*.map" \
    -or -name "*.hex" \
    | xargs rm -rf

find ../ -type d -empty -exec rm -rf {} \;
