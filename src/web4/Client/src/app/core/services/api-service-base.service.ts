import { Injectable, Inject, Optional, InjectionToken } from '@angular/core';
import { HttpHeaders, HttpClient, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable, throwError } from 'rxjs';

export const API_BASE_URL = new InjectionToken<string>('API_BASE_URL');

@Injectable({
  providedIn: 'root',
})
export abstract class ApiServiceBase {

  protected baseUrl: string;

  constructor(public _http: HttpClient, @Optional() @Inject(API_BASE_URL) baseUrl?: string) {
    this.baseUrl = baseUrl ?? "";
  }

  getRequest<T>(url: string, contentType: string = 'application/json', params?: HttpParams): Observable<T> {
    const headers = this.getHeaders(contentType);
    return this._http.get<T>(url, { headers, params: params });
  }

  putRequest<T>(url: string, body: T, contentType: string = 'application/json'): Observable<T> {
    const headers = this.getHeaders(contentType);
    return this._http.put<T>(url, body, { headers });
  }

  patchRequest<T>(url: string, body: T, contentType: string = 'application/json'): Observable<T> {
    const headers = this.getHeaders(contentType);
    return this._http.patch<T>(url, body, { headers });
  }

  postRequest<R, B>(url: string, body: B, contentType: string = 'application/json'): Observable<R> {
    const headers = this.getHeaders(contentType);
    return this._http.post<R>(url, body, { headers });
  }

  deleteRequest<T>(url: string, contentType: string = 'application/json'): Observable<T> {
    const headers = this.getHeaders(contentType);
    return this._http.delete<T>(url, { headers });
  }

  getHeaders(contentType: string): HttpHeaders {

    const headers = new HttpHeaders()
      .set('Accept', 'application/json')
      .set('Content-Type', contentType);

    return headers;
  }
}