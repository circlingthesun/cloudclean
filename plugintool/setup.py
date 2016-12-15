import os

from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
README = open(os.path.join(here, 'README.txt')).read()
CHANGES = open(os.path.join(here, 'CHANGES.txt')).read()

requires = [
    'mako',
    ]

setup(name='PluginTool',
      version='0.0',
      description='Cloudclean plugin tool',
      long_description=README + '\n\n' + CHANGES,
      classifiers=[
        "Development Status :: 3 - Alpha",
        "Programming Language :: Python",
        "Environment :: Console",
        "Topic :: Software Development :: Code Generators",
        ],
      author='',
      author_email='',
      url='',
      keywords='cloud clean',
      packages=find_packages(),
      include_package_data=True,
      zip_safe=False,
      install_requires=requires,
      entry_points="""\
      [console_scripts]
      createplugin = plugintool.createplugin:main
      """,
      )
