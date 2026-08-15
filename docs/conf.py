# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'DroneManager'
copyright = '2025, Dominik Mattern, Konstantin Bake'
author = 'Dominik Mattern, Konstantin Bake'
release = '0.3'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme',
]
autodoc_mock_imports = []  # "mujoco"
autodoc_member_order = 'bysource'
# autodoc_class_signature = 'separated' # Moves the class signatures into separate init function docu.
numpydoc_class_members_toctree = False
automodapi_toctreedirnm = 'generated'
automodsumm_inherited_members = True
autodoc_preserve_defaults = True
autoclass_content = 'class'

napoleon_google_docstring = True
#napoleon_use_ivar = True

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


source_suffix = {'.rst': 'restructuredtext'}

# The master toctree document.
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

autosummary_generate = False
