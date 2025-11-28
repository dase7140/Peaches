import os

# Folder you want to scan (use "." for current directory)
ROOT = "."

# Characters Windows does not allow in filenames
INVALID_CHARS = [":"]

def fix_name(name):
    new = name
    for c in INVALID_CHARS:
        new = new.replace(c, "-")
    return new

for folder, subfolders, files in os.walk(ROOT, topdown=False):
    # Rename files
    for file in files:
        if any(c in file for c in INVALID_CHARS):
            old_path = os.path.join(folder, file)
            new_file = fix_name(file)
            new_path = os.path.join(folder, new_file)

            print(f"Renaming file: {old_path} -> {new_path}")
            os.rename(old_path, new_path)

    # Rename folders
    for sub in subfolders:
        if any(c in sub for c in INVALID_CHARS):
            old_path = os.path.join(folder, sub)
            new_folder = fix_name(sub)
            new_path = os.path.join(folder, new_folder)

            print(f"Renaming folder: {old_path} -> {new_path}")
            os.rename(old_path, new_path)

print("Done! All invalid filenames have been fixed.")
