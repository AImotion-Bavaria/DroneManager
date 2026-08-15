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


import importlib
from typing import ClassVar, get_origin, get_type_hints


def _find_owner_class(qualified_name: str):
    """Find the class owning a fully qualified Sphinx member name."""
    parts = qualified_name.split(".")

    for i in range(len(parts) - 1, 0, -1):
        module_name = ".".join(parts[:i])

        try:
            obj = importlib.import_module(module_name)
        except ImportError:
            continue

        try:
            for part in parts[i:-1]:
                obj = getattr(obj, part)
        except AttributeError:
            continue

        if isinstance(obj, type):
            return obj

    return None


def _is_instance_attribute(cls: type, name: str) -> bool:
    """Return whether a class annotation describes an instance attribute."""
    for base in cls.__mro__:
        annotations = getattr(base, "__annotations__", {})

        if name not in annotations:
            continue

        try:
            annotation = get_type_hints(base, include_extras=True).get(name)
        except (NameError, TypeError):
            annotation = annotations[name]

        return get_origin(annotation) is not ClassVar

    return False


def skip_instance_attributes(app, what, name, obj, skip, options):
    """Prevent autodoc from duplicating documented instance attributes."""
    if what != "attribute":
        return skip

    owner = _find_owner_class(name)

    if owner is not None and _is_instance_attribute(owner, name.rsplit(".", 1)[-1]):
        return True

    return skip


def setup(app):
    app.connect("autodoc-skip-member", skip_instance_attributes)