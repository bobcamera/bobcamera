using bob_web4.Domain.Entities;

namespace bob_web4.Application.Recordings.Queries.GetRecordings;

public class RecordingDto
{
    public int Id { get; init; }

    public string? Filename { get; init; }

    private class Mapping : Profile
    {
        public Mapping()
        {
            CreateMap<Recording, RecordingDto>().ForMember(d => d.Filename, opt => opt.MapFrom(s => s.File));
        }
    }
}
