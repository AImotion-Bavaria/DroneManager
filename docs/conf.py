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


from typing import ClassVar, get_origin


_current_class = None
_instance_attributes = {}


def _is_classvar(annotation):
    """Return whether an annotation is a ClassVar."""
    return get_origin(annotation) is ClassVar


def _process_class_docstring(app, what, name, obj, options, lines):
    """Record instance attributes declared by each class."""
    global _current_class

    if what != "class":
        return

    _current_class = obj

    annotations = getattr(obj, "__annotations__", {})

    _instance_attributes[id(obj)] = {
        attribute_name
        for attribute_name, annotation in annotations.items()
        if not _is_classvar(annotation)
    }


def _skip_instance_attribute(app, what, name, obj, skip, options):
    """Skip annotated instance attributes from autodoc."""
    if what != "attribute":
        return skip

    if _current_class is None:
        return skip

    if name in _instance_attributes.get(id(_current_class), set()):
        return True

    return skip


def setup(app):
    app.connect("autodoc-process-docstring", _process_class_docstring)
    app.connect("autodoc-skip-member", _skip_instance_attribute)
