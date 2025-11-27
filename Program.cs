using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using opentk_learn;

var nativeWinSettings = new NativeWindowSettings()
{
    ClientSize = new Vector2i(1000, 600),
    Location = new Vector2i(370, 300),
    WindowBorder = WindowBorder.Resizable,
    WindowState = WindowState.Normal,
    Title = "Tiny3D",
    APIVersion = new Version(3, 3),
    Flags = ContextFlags.Default,
    Profile = ContextProfile.Compatability,
    API = ContextAPI.OpenGL
};

using Game game = new(GameWindowSettings.Default, nativeWinSettings);
game.Run();
