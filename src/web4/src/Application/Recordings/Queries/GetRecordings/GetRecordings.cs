using bob_web4.Application.Common.Interfaces;

namespace bob_web4.Application.Recordings.Queries.GetRecordings;

public record GetRecordingsQuery : IRequest<List<RecordingDto>>
{
    public string? Directory { get; init; }
}

public class GetRecordingsQueryHandler : IRequestHandler<GetRecordingsQuery, List<RecordingDto>>
{
    private readonly IApplicationDbContext _context;
    private readonly IMapper _mapper;

    public GetRecordingsQueryHandler(IApplicationDbContext context, IMapper mapper)
    {
        _context = context;
        _mapper = mapper;
    }

    public async Task<List<RecordingDto>> Handle(GetRecordingsQuery request, CancellationToken cancellationToken)
    {
        /*var list = new List<RecordingDto>();
        list.Add(new RecordingDto(){ Id = 1, Filename = "1.txt" });
        list.Add(new RecordingDto(){ Id = 2, Filename = "2.txt" });
        list.Add(new RecordingDto(){ Id = 3, Filename = "3.txt" });
        return await Task.FromResult(list);*/
        return await _context.Recordings
            .AsNoTracking()            
            .ProjectTo<RecordingDto>(_mapper.ConfigurationProvider)
            .OrderBy(x => x.Id)
            .ToListAsync(cancellationToken);
    }
}
