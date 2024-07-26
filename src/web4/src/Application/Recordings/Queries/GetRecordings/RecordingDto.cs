using bob_web4.Domain.Entities;

namespace bob_web4.Application.Recordings.Queries.GetRecordings;

public class RecordingDto
{
    public int Id { get; init; }

    public string? Filename { get; init; }
}
