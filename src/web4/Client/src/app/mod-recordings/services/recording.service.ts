import { HttpClient, HttpHeaders } from "@angular/common/http";
import { Injectable } from "@angular/core";
import { catchError, map, Observable } from "rxjs";

import { ApiServiceBase } from '../../mod-main/services/api-service-base.service'
import { RecordingQuery, RecordingDto } from '../models';

@Injectable({
  providedIn: 'root',
})
export class RecordingService extends ApiServiceBase {

  constructor(private http: HttpClient) {
    super(http);
  }

  deleteRecording(input: RecordingDto): Observable<any> {
    return super.deleteRequest<RecordingDto>(
      `${this.baseUrl}/api/recordings/${encodeURI(input.file)}`);
  }

  recordings(input: RecordingQuery): Observable<RecordingDto[]> {
    return super.getRequest<RecordingDto[]>(
        `${this.baseUrl}/api/recordings`);
  }
}