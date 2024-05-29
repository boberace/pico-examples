import os

def generate_html_response():
    html_file_path = os.path.join("data", "index.html")
    output_file_path = "html_response.c"
    
    try:
        with open(html_file_path, 'r') as html_file:
            lines = html_file.readlines()
        
        with open(output_file_path, 'w') as output_file:
            output_file.write('const char* html_response = \n')
            for line in lines:
                # Escape double quotes and strip the line
                escaped_line = line.replace('"', '\\"').strip()
                output_file.write(f'"{escaped_line}\\n"\n')
            output_file.write(';\n')
        
        print(f"Successfully generated {output_file_path} from {html_file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    generate_html_response()
