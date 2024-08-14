using System.Data.SqlTypes;
using bob_web4.Application.Common.Interfaces;

namespace bob_web4.Application.Vision.Queries.GetCamera;

public record GetCameraQuery : IRequest<CameraDto>
{
    public string? CameraName { get; init; }
}

public class GetCameraQueryHandler : IRequestHandler<GetCameraQuery, CameraDto>
{
    private readonly IApplicationDbContext _context;
    private readonly IMapper _mapper;

    public GetCameraQueryHandler(IApplicationDbContext context, IMapper mapper)
    {
        _context = context;
        _mapper = mapper;
    }

    public async Task<CameraDto> Handle(GetCameraQuery request, CancellationToken cancellationToken)
    {
        var camera = new CameraDto()
        {
            CameraName = request.CameraName,
            FPS = 30,
            Recording = DateTime.Now.Second % 2 == 0 ? true : false,
            RandomNumber = BetterRandom.NextInt()
        };

        return await Task.FromResult(camera);
    }
}

public static class BetterRandom
{
    private static readonly ThreadLocal<System.Security.Cryptography.RandomNumberGenerator> _crng = new ThreadLocal<System.Security.Cryptography.RandomNumberGenerator>(System.Security.Cryptography.RandomNumberGenerator.Create);
    private static readonly ThreadLocal<byte[]> _bytes = new ThreadLocal<byte[]>(() => new byte[sizeof(int)]);
    public static int NextInt()
    {
        if(_crng.Value != null && _bytes.Value != null)
        {
            _crng.Value.GetBytes(_bytes.Value);
            return BitConverter.ToInt32(_bytes.Value, 0) & int.MaxValue;
        }
        throw new ApplicationException("Both _crng.Value && _bytes.Value == null");
    }
    public static double NextDouble()
    {
        while (true)
        {
            long x = NextInt() & 0x001FFFFF;
            x <<= 31;
            x |= (long)NextInt();
            double n = x;
            const double d = 1L << 52;
            double q = n / d;
            if (q != 1.0)
                return q;
        }
    }
}
