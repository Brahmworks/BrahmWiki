# BrahmWiki
The following repo will be the master repo which will contain all the lib files.
Perfect! Now I understand your specific requirements for ESP32 and ATmega controllers with a clear embedded systems project structure. Here's the complete setup for your microcontroller library documentation system:

## Complete Setup for ESP32/ATmega Library Documentation System

### Step 1: Embedded Systems Repository Structure

**Create your repository** with this embedded systems-focused structure :[1][2]
```
your-embedded-repo/
├── lib/                           # All reusable libraries
│   ├── sensors/
│   │   ├── temperature_ds18b20/
│   │   │   ├── README.md
│   │   │   ├── src/
│   │   │   │   ├── ds18b20.h
│   │   │   │   └── ds18b20.c
│   │   │   └── examples/
│   │   │       ├── esp32_basic.c
│   │   │       ├── atmega328_basic.c
│   │   │       └── atmega2560_basic.c
│   │   ├── humidity_dht22/
│   │   │   ├── README.md
│   │   │   ├── src/
│   │   │   └── examples/
│   │   └── pressure_bmp280/
│   │       ├── README.md
│   │       ├── src/
│   │       └── examples/
│   ├── communication/
│   │   ├── wifi_esp32/
│   │   ├── uart_common/
│   │   └── i2c_common/
│   └── actuators/
│       ├── servo_control/
│       ├── stepper_motor/
│       └── dc_motor/
├── src/                           # Main project source files
│   ├── esp32/
│   │   ├── main.c
│   │   ├── config.h
│   │   └── platformio.ini
│   ├── atmega328/
│   │   ├── main.c
│   │   ├── config.h
│   │   └── Makefile
│   └── atmega2560/
│       ├── main.c
│       ├── config.h
│       └── Makefile
├── examples/                      # Complete project examples
│   ├── weather_station_esp32/
│   ├── home_automation_atmega328/
│   └── iot_sensor_node_atmega2560/
├── docs/
│   └── index.md
├── main.c                         # Template main file
├── mkdocs.yml
├── gen_embedded_docs.py
└── .github/
    └── workflows/
        └── embedded-docs.yml
```

### Step 2: MkDocs Configuration for Embedded Systems

**Create `mkdocs.yml`** optimized for embedded documentation :[3][4]
```yaml
# Embedded Systems Documentation Site
site_name: ESP32 & ATmega Library Collection
site_url: https://yourusername.github.io/embedded-library-repo
site_author: Your Name
site_description: Complete library collection for ESP32, ATmega328P, and ATmega2560 microcontrollers

# Repository Information
repo_url: https://github.com/yourusername/embedded-library-repo
repo_name: yourusername/embedded-library-repo
edit_uri: edit/main/

# Theme Configuration for Technical Documentation
theme:
  name: material
  
  palette:
    - scheme: slate
      primary: deep purple
      accent: purple
      toggle:
        icon: material/weather-sunny
        name: Switch to light mode
    - scheme: default
      primary: indigo
      accent: indigo
      toggle:
        icon: material/weather-night
        name: Switch to dark mode
  
  features:
    - navigation.tabs
    - navigation.tabs.sticky
    - navigation.sections
    - navigation.expand
    - navigation.top
    - search.highlight
    - search.share
    - content.code.copy
    - content.code.annotate
    - content.action.edit

  icon:
    repo: fontawesome/brands/github

# Plugins for Embedded Documentation
plugins:
  - search:
      lang: en
  - gen-files:
      scripts:
        - gen_embedded_docs.py

# Markdown Extensions for Code and Diagrams
markdown_extensions:
  - admonition
  - attr_list
  - def_list
  - md_in_html
  - toc:
      permalink: true
      title: On this page
  - pymdownx.arithmatex:
      generic: true
  - pymdownx.betterem
  - pymdownx.caret
  - pymdownx.details
  - pymdownx.emoji:
      emoji_index: !!python/name:material.extensions.emoji.twemoji
      emoji_generator: !!python/name:material.extensions.emoji.to_svg
  - pymdownx.highlight:
      anchor_linenums: true
      line_spans: __span
      pygments_lang_class: true
  - pymdownx.inlinehilite
  - pymdownx.keys
  - pymdownx.mark
  - pymdownx.smartsymbols
  - pymdownx.superfences:
      custom_fences:
        - name: mermaid
          class: mermaid
          format: !!python/name:pymdownx.superfences.fence_code_format
  - pymdownx.tabbed:
      alternate_style: true
  - pymdownx.tasklist:
      custom_checkbox: true

extra_css:
  - stylesheets/embedded.css

extra_javascript:
  - javascripts/embedded.js

# Extra Configuration for Embedded Systems
extra:
  social:
    - icon: fontawesome/brands/github
      link: https://github.com/yourusername
  generator: false
```

### Step 3: Advanced Documentation Generation Script

**Create `gen_embedded_docs.py`** for embedded systems :[5][6]
```python
#!/usr/bin/env python3
"""
Generate comprehensive embedded systems documentation
Supports ESP32, ATmega328P, and ATmega2560
"""
import os
import re
import shutil
from pathlib import Path
from typing import List, Dict

class EmbeddedDocsGenerator:
    def __init__(self):
        self.lib_dir = Path("lib")
        self.src_dir = Path("src")
        self.docs_dir = Path("docs")
        self.examples_dir = Path("examples")
        
        # Microcontroller configurations
        self.mcu_config = {
            'esp32': {
                'name': 'ESP32',
                'description': 'Dual-core 32-bit MCU with Wi-Fi & Bluetooth',
                'features': ['Wi-Fi', 'Bluetooth', '240MHz', '520KB RAM', '4MB Flash'],
                'voltage': '3.3V',
                'gpio_pins': '34',
                'analog_pins': '18'
            },
            'atmega328': {
                'name': 'ATmega328P',
                'description': '8-bit AVR microcontroller',
                'features': ['UART', 'SPI', 'I2C', '16MHz', '2KB SRAM', '32KB Flash'],
                'voltage': '5V',
                'gpio_pins': '23',
                'analog_pins': '8'
            },
            'atmega2560': {
                'name': 'ATmega2560',
                'description': '8-bit AVR microcontroller with more I/O',
                'features': ['4x UART', '2x SPI', 'I2C', '16MHz', '8KB SRAM', '256KB Flash'],
                'voltage': '5V',
                'gpio_pins': '86',
                'analog_pins': '16'
            }
        }

    def generate_all_docs(self):
        """Generate complete embedded documentation"""
        print("🔧 Generating embedded systems documentation...")
        
        # Clear existing docs
        self.clear_generated_docs()
        
        # Generate main documentation sections
        self.generate_main_index()
        self.generate_library_docs()
        self.generate_mcu_guides()
        self.generate_project_examples()
        self.generate_api_reference()
        
        print("✅ Documentation generation complete!")

    def clear_generated_docs(self):
        """Clear previously generated docs"""
        for item in self.docs_dir.glob("*"):
            if item.is_dir() and item.name not in ["stylesheets", "javascripts"]:
                shutil.rmtree(item)

    def generate_main_index(self):
        """Generate main documentation homepage"""
        with open(self.docs_dir / "index.md", "w") as f:
            f.write("# ESP32 & ATmega Library Collection\n\n")
            f.write("Complete embedded systems library collection with examples for ")
            f.write("ESP32, ATmega328P, and ATmega2560 microcontrollers.\n\n")
            
            f.write("## 🎯 Quick Start Guide\n\n")
            f.write("1. **Choose your microcontroller** from the supported platforms\n")
            f.write("2. **Browse available libraries** in the Library Reference\n")
            f.write("3. **Check examples** for your specific MCU\n")
            f.write("4. **Fork this repository** to start your project\n")
            f.write("5. **Modify `main.c`** according to your requirements\n\n")
            
            f.write("## 🔧 Supported Microcontrollers\n\n")
            for mcu_key, config in self.mcu_config.items():
                f.write(f"### {config['name']}\n\n")
                f.write(f"**{config['description']}**\n\n")
                f.write("| Specification | Value |\n")
                f.write("|---------------|-------|\n")
                f.write(f"| Operating Voltage | {config['voltage']} |\n")
                f.write(f"| GPIO Pins | {config['gpio_pins']} |\n")
                f.write(f"| Analog Pins | {config['analog_pins']} |\n")
                f.write(f"| Key Features | {', '.join(config['features'])} |\n\n")
            
            self.write_available_libraries(f)
            self.write_main_template(f)

    def write_available_libraries(self, f):
        """Write available libraries section"""
        f.write("## 📚 Available Library Categories\n\n")
        
        if self.lib_dir.exists():
            categories = [d for d in self.lib_dir.iterdir() if d.is_dir()]
            for category in sorted(categories):
                f.write(f"### {category.name.title()}\n\n")
                
                libraries = [d for d in category.iterdir() if d.is_dir()]
                for lib in sorted(libraries):
                    lib_name = lib.name.replace('_', ' ').title()
                    f.write(f"- **[{lib_name}](libraries/{category.name}/{lib.name}/index.md)**")
                    
                    # Try to extract description from README
                    readme_path = lib / "README.md"
                    if readme_path.exists():
                        description = self.extract_description(readme_path)
                        f.write(f" - {description}")
                    f.write("\n")
                f.write("\n")

    def write_main_template(self, f):
        """Write main.c template section"""
        f.write("## 🚀 Main Code Template\n\n")
        f.write("The main.c file serves as your project's entry point. ")
        f.write("Modify it according to your specific requirements:\n\n")
        
        main_c = Path("main.c")
        if main_c.exists():
            f.write("```
            f.write(main_c.read_text())
            f.write("\n```\n\n")
        else:
            # Write default template
            f.write("```
            f.write(self.generate_default_main_template())
            f.write("\n```\n\n")

    def generate_library_docs(self):
        """Generate documentation for each library"""
        if not self.lib_dir.exists():
            return
            
        lib_docs_dir = self.docs_dir / "libraries"
        lib_docs_dir.mkdir(exist_ok=True)
        
        # Process each category
        for category_dir in sorted(self.lib_dir.iterdir()):
            if not category_dir.is_dir():
                continue
                
            cat_docs_dir = lib_docs_dir / category_dir.name
            cat_docs_dir.mkdir(exist_ok=True)
            
            # Generate category index
            self.generate_category_index(category_dir, cat_docs_dir)
            
            # Process each library in category
            for lib_dir in sorted(category_dir.iterdir()):
                if not lib_dir.is_dir():
                    continue
                    
                self.generate_single_library_docs(lib_dir, cat_docs_dir)

    def generate_single_library_docs(self, lib_path: Path, cat_docs_dir: Path):
        """Generate documentation for a single library"""
        lib_name = lib_path.name
        lib_docs_dir = cat_docs_dir / lib_name
        lib_docs_dir.mkdir(exist_ok=True)
        
        print(f"📖 Processing library: {lib_name}")
        
        # Copy README as main library page
        readme_path = lib_path / "README.md"
        if readme_path.exists():
            content = readme_path.read_text()
            # Enhanced README with MCU-specific information
            enhanced_content = self.enhance_library_readme(content, lib_path)
            with open(lib_docs_dir / "index.md", "w") as f:
                f.write(enhanced_content)
        
        # Generate examples documentation
        self.generate_library_examples(lib_path, lib_docs_dir)
        
        # Generate API documentation
        self.generate_library_api(lib_path, lib_docs_dir)

    def generate_library_examples(self, lib_path: Path, lib_docs_dir: Path):
        """Generate examples documentation for library"""
        examples_dir = lib_path / "examples"
        if not examples_dir.exists():
            return
            
        docs_examples_dir = lib_docs_dir / "examples"
        docs_examples_dir.mkdir(exist_ok=True)
        
        # Create examples index
        with open(docs_examples_dir / "index.md", "w") as f:
            lib_name = lib_path.name.replace('_', ' ').title()
            f.write(f"# {lib_name} Examples\n\n")
            f.write("This section contains practical examples for different microcontrollers.\n\n")
            
            # Group examples by microcontroller
            mcu_examples = {'esp32': [], 'atmega328': [], 'atmega2560': [], 'common': []}
            
            for example_file in examples_dir.glob("*"):
                if example_file.suffix in ['.c', '.ino', '.cpp']:
                    if 'esp32' in example_file.name.lower():
                        mcu_examples['esp32'].append(example_file)
                    elif 'atmega328' in example_file.name.lower():
                        mcu_examples['atmega328'].append(example_file)
                    elif 'atmega2560' in example_file.name.lower():
                        mcu_examples['atmega2560'].append(example_file)
                    else:
                        mcu_examples['common'].append(example_file)
            
            # Write MCU-specific examples
            for mcu, files in mcu_examples.items():
                if files and mcu != 'common':
                    config = self.mcu_config.get(mcu, {})
                    f.write(f"## {config.get('name', mcu.upper())} Examples\n\n")
                    
                    for example_file in sorted(files):
                        self.write_example_file(f, example_file, docs_examples_dir)
            
            # Write common examples
            if mcu_examples['common']:
                f.write("## Universal Examples\n\n")
                for example_file in sorted(mcu_examples['common']):
                    self.write_example_file(f, example_file, docs_examples_dir)

    def write_example_file(self, f, example_file: Path, docs_examples_dir: Path):
        """Write individual example file documentation"""
        example_name = example_file.stem.replace('_', ' ').title()
        f.write(f"### {example_name}\n\n")
        
        # Determine language for syntax highlighting
        lang_map = {'.c': 'c', '.cpp': 'cpp', '.ino': 'cpp'}
        lang = lang_map.get(example_file.suffix, 'c')
        
        f.write(f"```
        content = example_file.read_text()
        
        # Extract comments for description
        lines = content.split('\n')
        description_lines = []
        for line in lines[:10]:  # Check first 10 lines
            if line.strip().startswith('//') or line.strip().startswith('/*'):
                description_lines.append(line.strip().lstrip('/*/ '))
        
        if description_lines:
            f.write(f"// {' '.join(description_lines)}\n//\n")
        
        f.write(content)
        f.write("\n```\n\n")

    def generate_mcu_guides(self):
        """Generate microcontroller-specific setup guides"""
        mcu_docs_dir = self.docs_dir / "microcontrollers"
        mcu_docs_dir.mkdir(exist_ok=True)
        
        for mcu_key, config in self.mcu_config.items():
            mcu_file = mcu_docs_dir / f"{mcu_key}.md"
            with open(mcu_file, "w") as f:
                f.write(f"# {config['name']} Setup Guide\n\n")
                f.write(f"**{config['description']}**\n\n")
                
                f.write("## Overview\n\n")
                f.write("| Specification | Details |\n")
                f.write("|---------------|----------|\n")
                f.write(f"| Architecture | {'32-bit Xtensa' if 'esp32' in mcu_key else '8-bit AVR'} |\n")
                f.write(f"| Operating Voltage | {config['voltage']} |\n")
                f.write(f"| GPIO Pins | {config['gpio_pins']} |\n")
                f.write(f"| Analog Inputs | {config['analog_pins']} |\n\n")
                
                self.write_mcu_setup_instructions(f, mcu_key, config)
                self.write_mcu_pinout_info(f, mcu_key, config)

    def write_mcu_setup_instructions(self, f, mcu_key: str, config: Dict):
        """Write setup instructions for specific MCU"""
        f.write("## Development Environment Setup\n\n")
        
        if mcu_key == 'esp32':
            f.write("### Arduino IDE Setup\n\n")
            f.write("1. Install Arduino IDE 2.0 or later\n")
            f.write("2. Add ESP32 board manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`\n")
            f.write("3. Install ESP32 board package via Board Manager\n")
            f.write("4. Select your ESP32 board from Tools > Board menu\n\n")
            
            f.write("### PlatformIO Setup\n\n")
            f.write("``````\n\n")
        else:
            f.write("### Arduino IDE Setup\n\n")
            f.write("1. Install Arduino IDE\n")
            f.write("2. Select Arduino Uno (ATmega328P) or Arduino Mega (ATmega2560)\n")
            f.write("3. Connect via USB and select correct COM port\n\n")
            
            f.write("### Atmel Studio Setup\n\n")
            f.write("1. Download and install Atmel Studio\n")
            f.write("2. Create new GCC C Executable Project\n")
            f.write(f"3. Select {config['name']} as target device\n")
            f.write("4. Configure programmer (AVRISP, USBasp, etc.)\n\n")

    def extract_description(self, readme_path: Path) -> str:
        """Extract description from README file"""
        try:
            content = readme_path.read_text()
            lines = content.split('\n')
            for line in lines[1:5]:  # Look in first few lines after title
                if line.strip() and not line.startswith('#'):
                    return line.strip()[:100] + "..." if len(line) > 100 else line.strip()
        except:
            pass
        return "Library documentation"

    def generate_default_main_template(self) -> str:
        """Generate default main.c template"""
        return '''/*
 * Embedded Systems Project Template
 * Supports: ESP32, ATmega328P, ATmega2560
 * 
 * Modify this file according to your project requirements
 */

#ifdef ESP32
    #include <Arduino.h>
    // ESP32 specific includes
#elif defined(__AVR_ATmega328P__)
    #include <avr/io.h>
    #include <util/delay.h>
    // ATmega328P specific includes
#elif defined(__AVR_ATmega2560__)
    #include <avr/io.h>
    #include <util/delay.h>
    // ATmega2560 specific includes
#endif

// Include your library headers here
// #include "lib/sensors/temperature_ds18b20/src/ds18b20.h"

void setup() {
    // Initialization code
    #ifdef ESP32
        Serial.begin(115200);
    #else
        // AVR initialization
    #endif
    
    // Initialize your libraries here
}

void loop() {
    // Main program logic
    
    #ifdef ESP32
        delay(1000);
    #else
        _delay_ms(1000);
    #endif
}

#ifndef ESP32
int main() {
    setup();
    while(1) {
        loop();
    }
    return 0;
}
#endif'''

    def enhance_library_readme(self, content: str, lib_path: Path) -> str:
        """Enhance library README with MCU compatibility info"""
        enhanced = content + "\n\n"
        enhanced += "## 🔧 Microcontroller Compatibility\n\n"
        
        examples_dir = lib_path / "examples"
        if examples_dir.exists():
            supported_mcus = []
            for example in examples_dir.glob("*"):
                if 'esp32' in example.name.lower():
                    supported_mcus.append('ESP32')
                elif 'atmega328' in example.name.lower():
                    supported_mcus.append('ATmega328P')
                elif 'atmega2560' in example.name.lower():
                    supported_mcus.append('ATmega2560')
            
            if supported_mcus:
                enhanced += f"**Tested on:** {', '.join(set(supported_mcus))}\n\n"
            else:
                enhanced += "**Compatible with:** ESP32, ATmega328P, ATmega2560\n\n"
        
        enhanced += "See the [examples](examples/index.md) section for MCU-specific implementation details.\n"
        return enhanced

if __name__ == "__main__":
    generator = EmbeddedDocsGenerator()
    generator.generate_all_docs()
```

### Step 4: GitHub Actions Workflow for Embedded Systems

**Create `.github/workflows/embedded-docs.yml`** :[7]
```yaml
name: Build Embedded Systems Documentation

on:
  push:
    branches: [ main, master ]
    paths: 
      - 'lib/**'
      - 'src/**' 
      - 'examples/**'
      - 'main.c'
      - 'docs/**'
      - 'mkdocs.yml'
  pull_request:
    branches: [ main, master ]
  workflow_dispatch:

permissions:
  contents: write
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build-embedded-docs:
    runs-on: ubuntu-latest
    
    steps:
    - name: Checkout Repository
      uses: actions/checkout@v4
      with:
        fetch-depth: 0
        
    - name: Setup Python
      uses: actions/setup-python@v5
      with:
        python-version: '3.11'
        
    - name: Install Dependencies
      run: |
        python -m pip install --upgrade pip
        pip install mkdocs-material
        pip install mkdocs-gen-files
        pip install pymdown-extensions
        
    - name: Generate Embedded Documentation
      run: |
        python gen_embedded_docs.py
        echo "📚 Generated documentation structure:"
        find docs -type f -name "*.md" | head -20
        
    - name: Validate Documentation
      run: |
        # Check for essential documentation files
        test -f docs/index.md || exit 1
        echo "✅ Documentation validation passed"
        
    - name: Build Documentation Site
      run: |
        mkdocs build --verbose --strict
        echo "🏗️ Built documentation site"
        ls -la site/
        
    - name: Upload Documentation Artifact
      uses: actions/upload-pages-artifact@v3
      with:
        path: ./site

  deploy:
    needs: build-embedded-docs
    if: github.ref == 'refs/heads/main'
    
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
      
    runs-on: ubuntu-latest
    
    steps:
    - name: Deploy to GitHub Pages
      id: deployment
      uses: actions/deploy-pages@v4
```

### Step 5: Library Template for Sensors

**Create `lib/sensors/sensor_template/README.md`** :[8][5]
```markdown
# Sensor Template Library

Brief description of the sensor functionality and its applications in embedded systems.

## 📋 Features

- Feature 1 (e.g., Temperature measurement range: -55°C to +125°C)
- Feature 2 (e.g., 12-bit resolution)
- Feature 3 (e.g., 1-Wire digital interface)

## 🔌 Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Sensor IC | DS18B20 (example) |
| Operating Voltage | 3.0V - 5.5V |
| Interface | 1-Wire digital |
| Pull-up Resistor | 4.7kΩ |

## 🔧 Wiring Diagrams

### ESP32 Connections
| Sensor Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VDD | 3.3V | Power |
| GND | GND | Ground |
| DQ | GPIO 4 | Data line |

### ATmega328P Connections  
| Sensor Pin | Arduino Pin | ATmega328P Pin |
|------------|-------------|----------------|
| VDD | 5V | VCC |
| GND | GND | GND |
| DQ | Digital 2 | PD2 |

### ATmega2560 Connections
| Sensor Pin | Arduino Pin | ATmega2560 Pin |
|------------|-------------|----------------|
| VDD | 5V | VCC |
| GND | GND | GND |
| DQ | Digital 2 | PE4 |

## 🚀 Quick Start

1. Wire the sensor according to your microcontroller
2. Include the library header in your project
3. Initialize the sensor
4. Read sensor data

## 📚 API Reference

### Functions

#### `sensor_init(pin)`
Initialize sensor on specified pin.

**Parameters:**
- `pin`: GPIO pin number for data line

**Returns:** `0` on success, `-1` on error

#### `sensor_read_temp()`
Read temperature value from sensor.

**Returns:** Temperature in Celsius as float

## 🔍 Example Usage

See the [examples](examples/index.md) section for complete implementation examples for each supported microcontroller.

## 📖 Troubleshooting

- **No sensor detected**: Check wiring and pull-up resistor
- **Incorrect readings**: Verify power supply voltage
- **Communication errors**: Check data line connection

## 📄 Datasheet References

- [Sensor Datasheet](link-to-datasheet)
- [Application Notes](link-to-app-notes)
```

This complete setup provides:

- **MCU-specific examples** for ESP32, ATmega328P, and ATmega2560[2][1]
- **Automatic navigation** based on your library folder structure[9]
- **Embedded systems focus** with hardware specifications and wiring diagrams[6][3]
- **Real-time updates** whenever you merge new sensor libraries[7]
- **Professional documentation** with code highlighting and technical details[4]

When you create a new sensor library and merge it to main, it automatically becomes a new tab in your documentation with complete setup guides for all three microcontrollers.[10][2]

[1](https://docs.arduino.cc/learn/contributions/arduino-creating-library-guide/)
[2](https://github.com/Mina267/Embedded-systems-projects)
[3](https://www.linkedin.com/advice/0/what-best-practices-documenting-maintaining-1c)
[4](https://www.semiconductor-digest.com/simplifying-technical-documentation-for-embedded-systems/)
[5](https://www.reddit.com/r/embedded/comments/nmyecj/embedded_software_documentation_very_confused_help/)
[6](https://www.cs.columbia.edu/~sedwards/classes/2025/4840-spring/sample-design-document.pdf)
[7](https://www.hatica.io/blog/automating-documentation-with-github-actions/)
[8](https://deepbluembedded.com/esp32-eeprom-library-tutorial-arduino/)
[9](https://visualgdb.com/documentation/projects/arduino/)
[10](https://www.electronicsforu.com/electronics-projects/hardware-diy/avr-projects-atmega-microcontroller)
[11](https://www.embeddedrelated.com/documents.php)
[12](https://www.bharathuniv.ac.in/colleges1/downloads/courseware_ece/notes/BEI605-%20Embedded-System.pdf)
[13](https://github.com/FracktalWorks/Project-Documentation-Template)
[14](https://www.engineersgarage.com/esp32-sd-card-emmc-filesystems/)
[15](https://duepublico2.uni-due.de/servlets/MCRFileNodeServlet/duepublico_derivate_00070068/Diss_Ulfat_Bunyadi.pdf)
[16](https://www.scribd.com/document/374393987/Documentation-Example)
[17](https://developer.espressif.com/blog/esp32-memory-map-101/)
[18](https://github.com/m3y54m/Embedded-Engineering-Roadmap)
[19](https://www.scribd.com/doc/38734447/8051-Micro-Controller-Project-Report)
[20](https://forum.arduino.cc/t/esp32-libraries-location/599643)
[21](https://creately.com/diagram/example/ih0qd6fu1/microcontroller-project-classic)
[22](https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/)
[23](https://www.scribd.com/document/850816482/13-3-24)
[24](https://www.st.com/en/microcontrollers-microprocessors/documentation.html)
[25](https://github.com/esp-arduino-libs)
[26](https://pallavaggarwal.in/technical-specification-for-an-embedded-product/)
[27](https://www.arduino.cc/en/Guide/libraries)
[28](https://docs.espressif.com/projects/arduino-esp32/en/latest/lib_builder.html)
[29](https://circuitdigest.com/avr-microcontroller-projects)
[30](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)
[31](https://circuitdigest.com/atmega32-projects)
[32](https://arduino.github.io/arduino-cli/1.3/library-specification/)
[33](https://nevonprojects.com/avr-microcontroller-based-projects/)
[34](https://community.platformio.org/t/using-esp-idf-library-within-the-arduino-framework-esp32/17226)
[35](https://atmega32-avr.com/atmega328-avr-based-projects-list-2/)
[36](https://researchdesignlab.com/atmega-mini-project-board.html)
[37](https://sriembeddedsolutions.com/embedded-system-using-avr-mcu-atmega-2560/)
