/**
 * Normalizers to adapt legacy/alternative backend shapes to the UI schema.
 * Keep the UI types stable even if the backend sends slightly different fields.
 */

export type LogSettings = {
  path: string;
  enabled: boolean;
  maxSize: number;
  retention: number; // days
};

// Alternate/legacy variant sometimes seen from backend or test data
export type LogSettingsAlt = {
  enabled: boolean;
  retentionDays: number;
  maxSize: number;
  path?: string;
};

export function normalizeLogSettings(x: LogSettings | LogSettingsAlt | undefined): LogSettings | undefined {
  if (!x) return undefined;
  const anyX: any = x as any;
  return {
    path: anyX.path ?? "",
    enabled: Boolean(anyX.enabled),
    maxSize: Number(anyX.maxSize ?? 0),
    retention: Number(anyX.retention ?? anyX.retentionDays ?? 0),
  };
}

export type NetworkA = { apiPort: number; wsPort: number; streamPort: number };
export type NetworkB = { apiPort: number; wsPort: number; enableCors: boolean; streamPort?: number };

export function normalizeNetwork(
  cfg: NetworkA | NetworkB | undefined,
  fallbackStreamPort?: number
): NetworkA | undefined {
  if (!cfg) return undefined;
  const anyC: any = cfg as any;
  const streamPort =
    typeof anyC.streamPort === "number"
      ? anyC.streamPort
      : (typeof fallbackStreamPort === "number" ? fallbackStreamPort : (Number(anyC.wsPort) || 0) + 1);

  return {
    apiPort: Number(anyC.apiPort),
    wsPort: Number(anyC.wsPort),
    streamPort: Number(streamPort),
  };
}

// Allowed system status union (map unknowns/offline to error)
export type AllowedStatus = "ok" | "degraded" | "error";
export function mapStatus(status: string | null | undefined): AllowedStatus {
  const s = String(status || "").toLowerCase();
  if (s === "ok" || s === "degraded" || s === "error") return s as AllowedStatus;
  return "error"; // treat "offline" or unknowns as error
}

// Safe numeric helper for nullable health readings
export function safeNumber(n: number | null | undefined, def = 0): number {
  return typeof n === "number" && !Number.isNaN(n) ? n : def;
}
