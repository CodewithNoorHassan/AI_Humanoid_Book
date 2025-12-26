#!/bin/bash

# Install dependencies
npm install

# Make sure the docusaurus binary has execute permissions
chmod +x node_modules/.bin/docusaurus || true

# Build the Docusaurus site
npx docusaurus build