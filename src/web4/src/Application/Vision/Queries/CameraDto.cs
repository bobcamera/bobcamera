namespace bob_web4.Application.Vision.Queries.GetCamera;

public class CameraDto
{
    public string? CameraName { get; init; }
    public int FPS { get; init; }
    public bool Recording { get; init; }
    public int RandomNumber { get; init; }
}