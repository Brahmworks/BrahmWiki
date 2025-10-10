# BrahmWiki: Embedded Systems Library Collection

Welcome to BrahmWiki, a comprehensive collection of libraries for embedded systems development, with a focus on ESP32, ATmega328P, and ATmega2560 microcontrollers. This repository is designed to be a centralized resource for reusable code, complete with automated documentation and examples.

## Features

- **Multi-Platform Support:** Libraries and examples for ESP32, ATmega328P, and ATmega2560.
- **Automated Documentation:** The documentation is automatically generated and deployed to GitHub Pages using `mkdocs` and GitHub Actions.
- **Organized Structure:** A clear and logical project structure that separates libraries, examples, and source code.
- **Easy to Extend:** A simple template for adding new sensor libraries and their documentation.

## Project Structure

The repository is organized into the following directories:

```
BrahmWiki/
├── .github/          # GitHub Actions workflows for automated deployment
├── docs/             # Generated documentation files
├── examples/         # Complete project examples
├── lib/              # Reusable libraries for sensors, actuators, etc.
├── src/              # Main project source files for each platform
├── gen_embedded_docs.py # Python script for generating documentation
├── mkdocs.yml        # Configuration file for the documentation site
└── README.md         # This file
```

## Getting Started

To get started with this repository, you can either clone it or fork it to your own GitHub account.

### Adding a New Library

To add a new library, follow these steps:

1.  **Create a new directory** for your library inside the appropriate category in the `lib/` directory (e.g., `lib/sensors/new_sensor`).
2.  **Add your source code** to a `src/` directory inside your new library's folder.
3.  **Add examples** for each supported platform to an `examples/` directory.
4.  **Create a `README.md`** file in your library's directory with documentation, wiring diagrams, and API references. You can use `lib/sensors/sensor_template/README.md` as a starting point.

### Viewing the Documentation

The documentation for this repository is automatically generated and deployed to GitHub Pages. You can view the live documentation at:

**https://Brahmworks.github.io/BrahmWiki/**

## Automated Documentation

This project uses `mkdocs` to generate a static documentation website from the Markdown files in this repository. The `gen_embedded_docs.py` script automates the process of creating documentation pages for each library, including its examples and API reference.

When you push changes to the `master` branch, a GitHub Actions workflow is triggered. This workflow automatically:

1.  Installs the necessary Python dependencies.
2.  Runs the `gen_embedded_docs.py` script to generate the documentation.
3.  Builds the `mkdocs` site.
4.  Deploys the site to the `gh-pages` branch, making it available at the link above.

## Contributing

Contributions are welcome! If you would like to contribute to this project, please follow these steps:

1.  Fork the repository.
2.  Create a new branch for your feature or bug fix.
3.  Make your changes and commit them with a descriptive message.
4.  Push your changes to your fork.
5.  Create a pull request to merge your changes into the main repository.

<!-- Trigger workflow -->



