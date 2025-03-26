from PIL import Image
import os
import math

image_no = 3
grid_dim = 800

def stitching_images(image_folder_path, output_img_path):
    print("[PC] Stitching images...")
    big_img = Image.new('RGB', (2400, 2400))

    # Get a list of all image files in the input folder
    image_files = [f for f in os.listdir(image_folder_path)]
    image_num = len(image_files)

    # Iterate through the image files and paste them onto the big image in a grid
    for i in range((image_no * image_no)):
        x = (i % image_no) * grid_dim
        y = (i // image_no) * grid_dim
        
        if i<image_num:
            img = Image.open(os.path.join(image_folder_path, image_files[i]))
            # Resize image to fit the grid cell size
            img = img.resize((grid_dim, grid_dim))
        else:
            gray_color = 128
            img = Image.new('L', (grid_dim, grid_dim ), gray_color)

        # Paste the image onto the big image
        big_img.paste(img, (x, y))

    # Save stitched image
    big_img.save(output_img_path)
    print(f'[PC] Stitched image saved to {output_img_path}')
    
if __name__ == "__main__":
    image_folder_path = r'images_result'
    stitching_images(r'images_result', r'images_result\test_stitched_image.jpg')