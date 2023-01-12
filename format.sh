clang-format -style=file src/* -i
files=$(find . -name "*.py"|grep -v "pybind11"|grep -v "\.\/ompl")
echo "format following python files $files"
python3 -m autoflake -i --remove-all-unused-imports --remove-unused-variables --ignore-init-module-imports $files
python3 -m isort $files
python3 -m black $files
python3 -m flake8 $files
