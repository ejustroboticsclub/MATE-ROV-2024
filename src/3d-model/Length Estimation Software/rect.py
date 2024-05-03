import pygame

ratio = 0

class Rectangle:
    def __init__(self, x, y, width, height, color):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        self.theta = 0
        self.image = pygame.Surface((width , height)) 
        self.image.fill(color)
        self.image.set_colorkey(pygame.Color('darkslategray')) 
        self.rect = self.image.get_rect()  
        self.rect.center = (x + width//2, y + height//2)
        self.text = ''
      
    def draw(self, screen):
        old_center = self.rect.center

        self.image = pygame.Surface((self.width , self.height))
        self.image.fill(self.color)
        self.image.set_colorkey(pygame.Color('darkslategray')) 
        self.image = pygame.transform.rotate(self.image , self.theta)  

        self.rect = self.image.get_rect()  
        self.rect.center = old_center
        screen.blit(self.image, self.rect)
        
        base_font = pygame.font.Font(None, 32) 
        text_surface = base_font.render(self.text, True, (255, 255, 255)) 
        text_rect = text_surface.get_rect()
        text_rect.x = old_center[0] - text_surface.get_width()//2
        text_rect.y = old_center[1] - text_surface.get_height()//2
        screen.blit(text_surface, text_rect)

        # update ratio
        global ratio
        if self.text != '':
            ratio = float(self.text) / self.width
       

    def drop(self, new_x, new_y):
        self.rect.x = new_x
        self.rect.y = new_y

    def move(self, dx, dy):
        self.rect.x += dx
        self.rect.y += dy

    def collidepoint(self, pos):
        return self.rect.collidepoint(pos)

    def change_width(self, dw):
        self.width += dw

    def change_height(self, dh):
        self.height += dh

    def change_angle(self, dtheta):
        self.theta += dtheta
        self.theta %= 360

    def print(self):
        global ratio
        if self.text != '':
            ratio = float(self.text) / self.width
            print(f"width: {self.width}, height: {self.height}, real width: {self.text}, ratio: {round(ratio, 2)}")
            return f"width: {self.width}, height: {self.height}, real width: {self.text}, ratio: {round(ratio, 2)}"
        else:
            print(f"width: {self.width}, height: {self.height}, real width: {round(self.width * ratio, 2)}")
            return f"width: {self.width}, height: {self.height}, real width: {round(self.width * ratio, 2)}"
