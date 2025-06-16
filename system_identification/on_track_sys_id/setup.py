from setuptools import setup, find_packages

setup(

    name='on_track_sys_"id',  # The name of your package
    version='0.1.0',         # Version of the package
    packages=find_packages('src'),  # Find all packages under 'src'
    package_dir={'': 'src'},        # Specify the root package directory
)