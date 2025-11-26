#!/bin/bash
find AP -type f \( -name "*.c" -o -name "*.h" \) -exec astyle --options=scripts/astylerc {} \;

