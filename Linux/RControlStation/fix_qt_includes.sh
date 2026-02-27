#!/bin/bash

# Script to fix Qt include paths in generated UI files
# Converts #include <QtWidgets/...> to #include <...>

echo "Fixing Qt include paths in UI files..."

# Fix UI files in the main directory
find . -name "ui_*.h" -not -path "./build/*" -exec echo "Fixing {}" \; -exec sed -i 's|#include <QtWidgets/|#include <|g' {} \;

# Fix UI files in build directories
echo "Fixing UI files in build directories..."
find ./build -name "ui_*.h" -exec echo "Fixing {}" \; -exec sed -i 's|#include <QtWidgets/|#include <|g' {} \;

echo "Qt include path fixes completed!"
