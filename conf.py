# docs/conf.py
import os
import sys

# -- Project information -----------------------------------------------------
project = 'RIG Documentation'
copyright = 'Robotics Interest Group'
author = 'SlyPredator'

# -- General configuration ---------------------------------------------------
extensions = [
    "myst_parser",           # For Markdown support
    "sphinx.ext.autodoc",    # For documenting code
    "sphinx.ext.napoleon",   # For Google/NumPy style docstrings
    "sphinx_copybutton",  
    "sphinx_design",   # Adds a copy button to code blocks
]

# Configure MyST-Parser to enable heading anchors
myst_enable_extensions = ["colon_fence", "deflist", "tasklist"]
myst_heading_anchors = 3

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', '.venv']

# -- Options for HTML output -------------------------------------------------
html_theme = "shibuya"

# Theme options are where Shibuya's specific features live
html_theme_options = {
    # "nav_links": [
    #     {"title": "Projects", "url": "index"},
    #     {"title": "GitHub", "url": "https://github.com/your-username"},
    # ],
    "accent_color": "blue",  # Can be: red, orange, yellow, green, blue, iris, purple
    "globaltoc_collapse": False,
    "globaltoc_expand_depth": 1,
}

# This ensures the sidebar shows up on every page
html_sidebars = {
    "**": [
        "sidebars/localtoc.html",
    ]
}

html_static_path = ['_static']
html_favicon = '_static/favicon.webp'