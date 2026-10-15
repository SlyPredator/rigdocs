# RIG Documentation

Documentation repository for the **Robotics Interest Group (RIGNITC)**. This site hosts tutorials, hardware configurations, and operational workflows for robotics platforms, sensor calibration, and state estimation pipelines.

## Local Setup

### 1. Prerequisites
- Python 3.10+
- Recommended: Virtual environment (`venv`)

### 2. Installation

```bash
# Create and activate a virtual environment
python -m venv .venv

# Linux / macOS:
source .venv/bin/activate

# Windows (PowerShell):
.\.venv\Scripts\Activate.ps1

# Install required dependencies
pip install -r requirements.txt
```

### 3. Running Locally

#### Development Server (Live Reload)

Runs a local development server that watches for file changes and refreshes the browser automatically:

```bash
sphinx-autobuild . _build/html --port 8000
```

Open `http://127.0.0.1:8000` in your browser.

#### Static HTML Build

Build static files to preview the production bundle:

```bash
# Linux / macOS:
make html

# Windows:
.\make.bat html
```

The generated HTML files are located in `_build/html/`.

## Repository Structure

```text
├── .github/workflows/    # GitHub Pages deployment workflow
├── _static/              # Custom scripts, CSS, and favicon
├── content/              # Documentation source articles (Markdown)
│   ├── kalibr/           # Camera-IMU calibration guide
│   ├── m2/               # Svan M2 quadruped guides (etiquette, DDS, control, sim)
│   ├── rovio/            # Robust Visual Inertial Odometry guide
│   └── test/             # Markdown rendering tests
├── conf.py               # Sphinx theme and build configuration
├── index.rst             # Root landing page and table of contents
└── requirements.txt      # Pinned Python dependencies
```
