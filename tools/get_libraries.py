#!/usr/bin/env python3

import subprocess

def get_running_containers():
    """Get a list of running Docker containers."""
    result = subprocess.run(['docker', 'ps', '--format', '{{.ID}}'], stdout=subprocess.PIPE)
    container_ids = result.stdout.decode('utf-8').strip().split('\n')
    return container_ids if container_ids[0] else []

def get_container_name(container_id):
    """Get the name of a container from its ID."""
    result = subprocess.run(['docker', 'inspect', container_id, '--format', '{{.Name}}'], stdout=subprocess.PIPE)
    return result.stdout.decode('utf-8').strip().lstrip('/')

def install_lsof(container_id):
    """Install lsof inside the container."""
    print(f"Installing lsof in container {container_id}...")
    # Update package lists
    subprocess.run(['docker', 'exec', container_id, 'apt-get', 'update'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # Install lsof
    subprocess.run(['docker', 'exec', container_id, 'apt-get', 'install', '-y', 'lsof'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def get_container_processes(container_id):
    """Get the list of running processes in the container."""
    try:
        result = subprocess.run(['docker', 'exec', container_id, 'ps', '-eo', 'pid,comm'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.stderr:
            print(f"Error fetching processes from container {container_id}: {result.stderr.decode('utf-8')}")
            return []
        process_lines = result.stdout.decode('utf-8').strip().split('\n')[1:]  # Skip the header
        return [line.strip().split(None, 1) for line in process_lines]  # Split into PID and command
    except Exception as e:
        print(f"Error: {e}")
        return []

def get_libraries_for_process(container_id, pid):
    """Use lsof to get shared libraries used by the process."""
    try:
        cmd = f"lsof -p {pid} | grep '.so'"
        result = subprocess.run(['docker', 'exec', container_id, 'sh', '-c', cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.stderr and result.stderr.decode('utf-8'):
            return []
        libraries = result.stdout.decode('utf-8').strip().split('\n')
        return libraries if libraries[0] else []
    except Exception as e:
        print(f"Error fetching libraries for process {pid}: {e}")
        return []

def parse_libraries(libraries):
    """Parse the lsof output to extract the library file paths."""
    paths = []
    for lib in libraries:
        parts = lib.split()
        if len(parts) > 8:  # The path should be at the end of the line
            path = parts[-1]
            if path.endswith('.so') or '.so.' in path:
                paths.append(path)
    return paths

def write_copy_commands(outfile, container_name, libraries):
    """Write Docker COPY commands for libraries to a text file."""
    outfile.write(f"# Docker COPY commands for container {container_name}\n")
    for lib in sorted(set(libraries)):
        outfile.write(f"COPY --from=build-env {lib} {lib}\n")
    outfile.write("\n")

def main():
    output_file = "docker_copy_commands.txt"
    containers = get_running_containers()
    
    with open(output_file, 'w') as outfile:
        if not containers:
            print("No running containers found.")
            return
        
        for container_id in containers:
            container_name = get_container_name(container_id)
            print(f"\nContainer: {container_name} (ID: {container_id})")
            
            # Install lsof in the container
            install_lsof(container_id)
            
            processes = get_container_processes(container_id)
            if not processes:
                print(f"No processes found in container {container_name}.")
                continue
            
            all_libraries = []
            for pid, command in processes:
                print(f"  Process: {command} (PID: {pid})")
                libraries = get_libraries_for_process(container_id, pid)
                if libraries:
                    print(f"    Shared Libraries:")
                    paths = parse_libraries(libraries)
                    all_libraries.extend(paths)
                    for lib in paths:
                        print(f"      {lib}")
                else:
                    print(f"    No shared libraries found.")
            
            if all_libraries:
                write_copy_commands(outfile, container_name, all_libraries)
            else:
                print(f"No shared libraries found for container {container_name}.")

if __name__ == "__main__":
    main()
