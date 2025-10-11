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
        # self.generate_main_index()
        self.generate_library_docs()
        self.generate_mcu_guides()
        # self.generate_project_examples()
        # self.generate_api_reference()
        
        print("✅ Documentation generation complete!")

    def clear_generated_docs(self):
        """Clear previously generated docs"""
        for item in self.docs_dir.glob("*"):
            if item.is_dir() and item.name not in ["stylesheets", "javascripts"]:
                shutil.rmtree(item)

    def generate_main_index(self):
        """Generate main documentation homepage"""
        with open(self.docs_dir / "index.md", "w", encoding="utf-8") as f:
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
            f.write("```c\n")
            f.write(main_c.read_text(encoding="utf-8"))
            f.write("\n```\n\n")
        else:
            # Write default template
            f.write("```c\n")
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
            # self.generate_category_index(category_dir, cat_docs_dir)
            
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
            content = readme_path.read_text(encoding="utf-8")
            # Enhanced README with MCU-specific information
            enhanced_content = self.enhance_library_readme(content, lib_path)
            with open(lib_docs_dir / "index.md", "w", encoding="utf-8") as f:
                f.write(enhanced_content)
        
        # Generate examples documentation
        self.generate_library_examples(lib_path, lib_docs_dir)
        
        # Generate API documentation
        # self.generate_library_api(lib_path, lib_docs_dir)

    def generate_library_examples(self, lib_path: Path, lib_docs_dir: Path):
        """Generate examples documentation for library"""
        examples_dir = lib_path / "examples"
        if not examples_dir.exists():
            return
            
        docs_examples_dir = lib_docs_dir / "examples"
        docs_examples_dir.mkdir(exist_ok=True)
        
        # Create examples index
        with open(docs_examples_dir / "index.md", "w", encoding="utf-8") as f:
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
        
        f.write(f"```{lang}\n")
        content = example_file.read_text(encoding="utf-8")
        
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
            with open(mcu_file, "w", encoding="utf-8") as f:
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
                # self.write_mcu_pinout_info(f, mcu_key, config)

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
            f.write("```ini\n[env:esp32dev]\nplatform = espressif32\nboard = esp32dev\nframework = arduino\n```\n\n")
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
            content = readme_path.read_text(encoding="utf-8")
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
