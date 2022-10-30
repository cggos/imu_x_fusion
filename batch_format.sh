#!/usr/bin/env bash

find . -regex '.*\.\(cpp\|cc\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
