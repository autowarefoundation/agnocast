import sys
import os
import re
import em

def camel_to_snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def main():
    if len(sys.argv) < 7:
        print("Usage: python generate_bridge_plugin.py <input_list_file> <output_directory> <output_deps_file> <output_clean_list_file> <template_file> <direction>")
        print("       <direction> must be 'r2a' or 'a2r'")
        return 1

    input_file = sys.argv[1]
    output_dir = sys.argv[2]
    output_deps_file = sys.argv[3]
    output_clean_list_file = sys.argv[4]
    template_file = sys.argv[5]
    direction = sys.argv[6]  # 'r2a' or 'a2r'

    os.makedirs(output_dir, exist_ok=True)

    try:
        with open(input_file, 'r') as f:
            message_types = [line.strip() for line in f if '/' in line.strip()]
    except FileNotFoundError:
        print(f"[Warning] Input file not found: {input_file}. No plugins will be generated.", file=sys.stderr)
        message_types = []

    package_names = set()

    for msg_type in message_types:
        pkg_name = msg_type.split('/')[0]
        package_names.add(pkg_name)

        flat_type = msg_type.replace('/', '_')
        safe_filename = f"register_{direction}_{flat_type}.cpp"
        output_cpp_path = os.path.join(output_dir, safe_filename)

        cpp_type = msg_type.replace('/', '::')
        
        parts = msg_type.split('/')
        parts[-1] = camel_to_snake(parts[-1])
        header_path = "/".join(parts) + ".hpp"
        
        function_name = f"create_{direction}_bridge"

        data = {
            'msg_type': msg_type,
            'cpp_type': cpp_type,
            'header_path': header_path,
            'function_name': function_name
        }

        with open(output_cpp_path, 'w') as f:
            with open(template_file, 'r') as temp_f:
                interpreter = em.Interpreter(output=f, globals=data)
                interpreter.file(temp_f)
                interpreter.shutdown()

    with open(output_clean_list_file, 'w') as f:
        for msg_type in message_types:
            f.write(f"{msg_type}\n")

    sorted_pkg_names = sorted(list(package_names))
    with open(output_deps_file, 'w') as f:
        for pkg_name in sorted_pkg_names:
            f.write(f"{pkg_name}\n")

    return 0

if __name__ == '__main__':
    sys.exit(main())
