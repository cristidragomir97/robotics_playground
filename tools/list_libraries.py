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

def main():
    output_file = "container_libraries.txt"
    all_libraries = set()
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
            
            for pid, command in processes:
                print(f"  Process: {command} (PID: {pid})")
                libraries = get_libraries_for_process(container_id, pid)
                all_libraries.update(libraries)
                if libraries:
                    print(f"    Shared Libraries:")
                    for lib in libraries:
                        print(f"      {lib}")
                else:
                    print(f"    No shared libraries found.")
            
            outfile.write(f"{container_name}\n")
            for lib in sorted(all_libraries):
                outfile.write(f"{lib}\n")
            outfile.write("\n\n")  # Add a few blank lines between containers

        

if __name__ == "__main__":
    main()
