# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'newton-esp-firmware'
copyright = '2025, TheNewtonCapstone'
author = 'TheNewtonCapstone'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
html_theme = 'sphinx_rtd_theme' 

# Breathe Configuration
extensions = [
    'sphinx.ext.autodoc',
    'breathe'
]

breathe_projects = { "newton-esp-firmware": "doxygen/xml" }
breathe_default_project = "newton-esp-firmware"


import subprocess

# Generate Doxygen XML
subprocess.call('doxygen Doxyfile', shell=True)

# Generate Sphinx pages
subprocess.call('python generate_rst.py', shell=True)