
import importlib.util
import slicer
dependencies = {
    "serial": "pyserial",
    "yaml": "PyYAML",
    "lxml": "lxml",
}

def check_dependency_installed(import_name, module_name_and_version):
    """
    Checks if a package is installed with the correct version.
    """
    if "==" in module_name_and_version:
        module_name, module_version = module_name_and_version.split("==")
    else:
        module_name = module_name_and_version
        module_version = None

    spec = importlib.util.find_spec(import_name)
    if spec is None:
        # Not installed
        return False

    if module_version is not None:
        import importlib.metadata as metadata
        try:
            version = metadata.version(module_name)
            if version != module_version:
                # Version mismatch
                return False
        except metadata.PackageNotFoundError:
            print(f"Could not determine version for {module_name}.")

    return True

def pip_install_wrapper(command):
    """
    Installs pip packages.
    """
    slicer.util.pip_install(command)

def install_dependencies():
    """
    Checks for (and installs if needed) python packages needed by the module.
    """
    for dependency in dependencies:
        if check_dependency_installed(dependency, dependencies[dependency]):
            continue
        
        pip_install_wrapper(dependencies[dependency])
            
        
