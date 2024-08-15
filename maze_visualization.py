from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle, Line
from kivy.core.window import Window
from kivy.clock import Clock


class MazeWidget(Widget):
    def __init__(self, **kwargs):
        super(MazeWidget, self).__init__(**kwargs)

        with self.canvas.before:
            Color(0, 0, 0)
            self.rect = Rectangle(size=self.size, pos=self.pos)
        self.bind(size=self.update_rect, pos=self.update_rect)

        self.path = ""
        self.optimized_path = ""
        self.path_index = 0
        self.end_square_size = 50  # Size of the end square
        self.direction = 3  # Initial direction: 0 = Right

        self.screen_width = Window.width
        self.screen_height = Window.height
        print(f"Screen size: {self.screen_width} x {self.screen_height}")

    def update_rect(self, instance, value):
        self.rect.size = instance.size
        self.rect.pos = instance.pos

    def draw_path_step(self, dt):
        if self.path_index < len(self.path):
            move = self.path[self.path_index]
            old_x, old_y = self.x, self.y
            if move == 'R':
                self.direction = (self.direction + 1) % 4  # Turn right
            elif move == 'L':
                self.direction = (self.direction - 1) % 4  # Turn left
            elif move == 'F':
                if self.direction == 0:  # Facing right
                    self.x += 50
                elif self.direction == 1:  # Facing down
                    self.y -= 50
                elif self.direction == 2:  # Facing left
                    self.x -= 50
                elif self.direction == 3:  # Facing up
                    self.y += 50
            elif move == 'B':
                if self.direction == 0:  # Facing right
                    self.x -= 50
                    self.direction = 2
                elif self.direction == 1:  # Facing down
                    self.y += 50
                    self.direction = 3
                elif self.direction == 2:  # Facing left
                    self.x += 50
                    self.direction = 0
                elif self.direction == 3:  # Facing up
                    self.y -= 50
                    self.direction = 1
            # Draw the segment
            with self.canvas:
                Color(1, 1, 1)  # White path color
                Line(points=[old_x, old_y, self.x, self.y], width=2)
            self.path_index += 1
        else:
            # Draw the end square
            self.draw_end_square()
            Clock.unschedule(self.draw_path_step)
            Clock.schedule_once(self.start_drawing_optimized_path, 2)

    def draw_optimized_path_step(self, dt):
        if self.path_index < len(self.optimized_path):
            move = self.optimized_path[self.path_index]
            old_x, old_y = self.x, self.y
            if move == 'R':
                self.direction = (self.direction + 1) % 4  # Turn right
            elif move == 'L':
                self.direction = (self.direction - 1) % 4  # Turn left
            elif move == 'F':
                if self.direction == 0:  # Facing right
                    self.x += 50
                elif self.direction == 1:  # Facing down
                    self.y -= 50
                elif self.direction == 2:  # Facing left
                    self.x -= 50
                elif self.direction == 3:  # Facing up
                    self.y += 50
            elif move == 'B':
                if self.direction == 0:  # Facing right
                    self.x -= 50
                    self.direction = 2
                elif self.direction == 1:  # Facing down
                    self.y += 50
                    self.direction = 3
                elif self.direction == 2:  # Facing left
                    self.x += 50
                    self.direction = 0
                elif self.direction == 3:  # Facing up
                    self.y -= 50
                    self.direction = 1
            with self.canvas:
                Color(0, 0, 1)
                Line(points=[old_x, old_y, self.x, self.y], width=2)
            self.path_index += 1
        else:
            Clock.unschedule(self.draw_optimized_path_step)
            self.readd_ui_elements()

    def draw_path(self, path, optimized_path):
        # Reset position and index

        self.screen_width = Window.width
        self.screen_height = Window.height
        self.x, self.y = self.screen_width / 2, 150
        self.path_index = 0
        self.direction = 3  # Reset direction to up
        self.path = ""
        self.optimized_path = ""
        for i in path:
            self.path += i
            self.path += "FFF" if i != "B" else "FF"

        for i in optimized_path:
            self.optimized_path += i
            self.optimized_path += "FFF" if i != "B" else "FF"
        print(self.path)
        print(self.optimized_path)
        # Clear existing drawings
        self.canvas.clear()
        # Set the background again after clearing
        with self.canvas.before:
            Color(0, 0, 0)
            self.rect = Rectangle(size=self.size, pos=self.pos)
        Clock.schedule_interval(self.draw_path_step, 0.2)

    def start_drawing_optimized_path(self, dt):
        self.x, self.y = self.screen_width / 2, 150
        self.path_index = 0
        self.direction = 3  # Reset direction to right
        Clock.schedule_interval(self.draw_optimized_path_step, 0.2)  # Adjust time interval as needed

    def draw_end_square(self):
        with self.canvas:
            Color(0, 0, 1)
            Rectangle(pos=(self.x - self.end_square_size / 2, self.y - self.end_square_size / 2),
                      size=(self.end_square_size, self.end_square_size))

    def readd_ui_elements(self):
        # Readd the TextInput and Button below the maze
        app = App.get_running_app()
        app.layout.clear_widgets()
        app.layout.add_widget(self)
        app.layout.add_widget(app.path_input)
        app.layout.add_widget(app.draw_button)


class MazeVisualizationApp(App):
    def build(self):
        self.maze_widget = MazeWidget()
        self.layout = BoxLayout(orientation='vertical')

        # Text input for path
        self.path_input = TextInput(hint_text='Enter Path and Optimized Path (e.g., LBSLLRSRRBR;RLLRSRS).',
                                    size_hint=(1, None), height=50, multiline=False)

        self.draw_button = Button(text='Draw Maze', size_hint=(1, None), height=50)
        self.draw_button.bind(on_press=self.draw_maze)

        self.layout.add_widget(self.maze_widget)
        self.layout.add_widget(self.path_input)
        self.layout.add_widget(self.draw_button)

        return self.layout

    def draw_maze(self, instance):
        paths = self.path_input.text.split(';')
        if len(paths) == 2:
            # Start each path with 'S' so it could draw a forward line at the beginning of the maze
            path = "S" + paths[0].strip()
            optimized_path = "S" + paths[1].strip()
            self.maze_widget.draw_path(path, optimized_path)
        else:
            print("Invalid input format. Please enter Path and Optimized Path separated by ';'.")


if __name__ == '__main__':
    MazeVisualizationApp().run()
