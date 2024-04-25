from rect import *
import os

# user-defined parameters
Width, Height = 1000, 600
target_img_width = 800  
Rect_color = (255, 0, 0)
data_folder_path = "data"

Center = (Width // 2, Height // 2)
arrow_path = os.path.join("assets", "arrow.png")
img_paths = []
for img in os.listdir(data_folder_path):
    img_paths.append(os.path.join(data_folder_path,img))

Rectangles = []

dx, dy, dw, dh, dtheta = 6, 6, 6, 2, 2

def initialize():
    pygame.init()
    scrn = pygame.display.set_mode((Width, Height))
    scrn.fill(pygame.Color('darkslategray'))
    pygame.display.set_caption('3D modeling Task: length estimation')
    return scrn


scrn = initialize()

l_arrow_img = pygame.image.load(arrow_path).convert_alpha()
l_arrow_img = pygame.transform.scale(l_arrow_img, (100,100))

r_arrow_img = pygame.transform.flip(l_arrow_img, True, False)

arrow_dim = (l_arrow_img.get_width(), l_arrow_img.get_height())
l_arrow_pos = (Width - (arrow_dim[0] + 5) , Center[1] - (arrow_dim[1] // 2))
r_arrow_pos = (5 , Center[1] - (arrow_dim[1] // 2))

l_arrow_rect = l_arrow_img.get_rect()
r_arrow_rect = r_arrow_img.get_rect()
l_arrow_rect.x, l_arrow_rect.y = l_arrow_pos
r_arrow_rect.x, r_arrow_rect.y = r_arrow_pos

i = 0
img = pygame.image.load(img_paths[i]).convert()
aspect_ratio = img.get_width() /  img.get_height()
new_height = int(target_img_width / aspect_ratio)
img_dim = (target_img_width, new_height)
img = pygame.transform.scale(img, img_dim)

img_start_Pos = (Center[0] - (img_dim[0] // 2), Center[1] - (img_dim[1] // 2))

rect_x = 30
rect_y = 30
rect_w = 60
rect_h = 10
std_rectangle = Rectangle(rect_x, rect_y, rect_w, rect_h, Rect_color)
last_rect = std_rectangle
# Rectangles.append(std_rectangle)

p1 = (1, 1)
p2 = (1, 1)
mouse_pos = (1, 1)
wh_p = 1

# Font initialization
font = pygame.font.SysFont('Arial', 30) 
title_surface = font.render('Length Estimation Software', True, pygame.Color('white'))  
font = pygame.font.SysFont('Arial', 24)  
copyright_surface = font.render('Made by Yasser', True, pygame.Color('white')) 


def scrn_update(i):
    img = pygame.image.load(img_paths[i]).convert()
    aspect_ratio = img.get_width() /  img.get_height()
    new_height = int(target_img_width / aspect_ratio)
    img_dim = (target_img_width, new_height)
    img = pygame.transform.scale(img, img_dim)

    img_start_Pos = (Center[0] - (img_dim[0] // 2), Center[1] - (img_dim[1] // 2))

    scrn.fill(pygame.Color('darkslategray'))
    scrn.blit(img, img_start_Pos)
    scrn.blit(l_arrow_img, l_arrow_pos)
    scrn.blit(r_arrow_img, r_arrow_pos)
    for rectangle in Rectangles:
        rectangle.draw(scrn)

    text_rect = copyright_surface.get_rect(bottomright=(Width - 10, Height - 10))
    scrn.blit(copyright_surface, text_rect)
    text_rect = title_surface.get_rect(center=(Width//2 , 25))
    scrn.blit(title_surface, text_rect)

    info_surface = font.render(info, True, pygame.Color('white')) 
    text_rect = info_surface.get_rect(bottomleft=(10 , Height - 10))
    scrn.blit(info_surface, text_rect)
    pygame.display.flip()
    

# Flag to indicate whether the rectangle is being dragged
dragging = False
status = True
info = ''
while status:
    scrn_update(i)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            status = False
        else:
            if event.type == pygame.MOUSEBUTTONDOWN:
                if l_arrow_rect.collidepoint(event.pos):
                    i+=1
                    if i == len(img_paths):
                        i = 0
                
                if r_arrow_rect.collidepoint(event.pos):
                    i-=1
                    if i < 0: 
                        i = len(img_paths)- 1

                for rectangle in Rectangles:
                    if rectangle.collidepoint(event.pos):
                        dragging = True
                        mouse_x, mouse_y = event.pos
                        offset_x = rectangle.rect.x - mouse_x
                        offset_y = rectangle.rect.y - mouse_y
                        last_rect = rectangle
            elif event.type == pygame.MOUSEBUTTONUP:
                dragging = False
            elif event.type == pygame.MOUSEMOTION:
                mouse_pos = event.pos
                if dragging:
                    mouse_x, mouse_y = event.pos
                    last_rect.drop(mouse_x + offset_x, mouse_y + offset_y)
            elif event.type == pygame.KEYDOWN:
                
                if event.key == pygame.K_BACKSPACE: 
                        last_rect.text = last_rect.text[:-1]  
                if event.key == pygame.K_DELETE: 
                        last_rect.text = ''  
                elif '0' <= event.unicode < '9' :
                        last_rect.text += event.unicode

                if event.key == pygame.K_d:
                    last_rect.change_width(dw)   # Increase width by 5 pixels
                elif event.key == pygame.K_a:
                    last_rect.change_width(-dw)  # Decrease width by 5 pixels
                elif event.key == pygame.K_w:
                    last_rect.change_height(dh)   # Increase height by 5 pixels
                elif event.key == pygame.K_s:
                    last_rect.change_height(-dh)  # Decrease height by 5 pixels
                elif event.key == pygame.K_q:
                    last_rect.change_angle(dh)   
                elif event.key == pygame.K_e:
                    last_rect.change_angle(-dh)
                elif event.key == pygame.K_RIGHT:
                    last_rect.move(dx, 0) 
                elif event.key == pygame.K_LEFT:
                    last_rect.move(-dx, 0)  
                elif event.key == pygame.K_UP:
                    last_rect.move(0, -dy) 
                elif event.key == pygame.K_DOWN:
                    last_rect.move(0, dy)   
                elif event.key == pygame.K_PAGEUP:
                    dtheta += 1
                    dx = dy = dw = dtheta * 3
                    dh = dtheta 
                  
                elif event.key == pygame.K_PAGEDOWN:
                    if dtheta != 0:
                        dtheta -= 1
                        dx = dy = dw = dtheta * 3
                        dh = dtheta 
                
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_n:
                    if wh_p == 1:
                        p1 = mouse_pos
                    else:
                        p2 = mouse_pos
                        new_rectangle = Rectangle(min(p1[0], p2[0]), min(p1[1], p2[1]), abs(p2[0] - p1[0]) , abs(p2[1] - p1[1]), Rect_color)
                        last_rect = new_rectangle
                        Rectangles.append(new_rectangle)
                    wh_p += 1
                    wh_p %= 2

                elif event.key == pygame.K_p:
                    info = last_rect.print()

                elif event.key == pygame.K_o:
                    i+=1
                    if i == len(img_paths):
                        i = 0
                elif event.key == pygame.K_u:
                    i-=1
                    if i < 0: 
                        i = len(img_paths)- 1
            
                


pygame.quit()
