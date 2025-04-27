from pynput import keyboard

def on_press(key):
    print(f"Tecla presionada: {key}")

listener = keyboard.Listener(on_press=on_press)
listener.start()

input("Presiona algunas teclas... (Ctrl+C para salir)\n")
