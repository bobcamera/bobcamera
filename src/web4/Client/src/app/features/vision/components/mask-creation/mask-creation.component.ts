import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input, Output, EventEmitter, ViewChild, ElementRef, AfterViewInit, Renderer2 } from '@angular/core';
import { Subject, Observable } from 'rxjs';

import { takeUntil } from 'rxjs/operators';

import { Point } from '../../models';

@Component({
  selector: 'bob-mask-creation',
  templateUrl: './mask-creation.component.html',
  styleUrls: ['./mask-creation.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MaskCreationComponent implements OnInit, OnDestroy, AfterViewInit {

  protected _ngUnsubscribe$: Subject<void> = new Subject<void>();
  
  @Input() ImageStream$: Observable<string>;
  @Input() EditMode$: Observable<boolean>;

  @Output() ConvasDimentionsChanged = new EventEmitter();

  @ViewChild('streamContainer', {static: false}) protected streamContainer: ElementRef;
  @ViewChild('imageSubscription', {static: false}) protected imageSubscription: ElementRef;
  @ViewChild('maskCanvas', {static: false}) protected maskCanvas: ElementRef;

  protected _maskCanvasCtx: CanvasRenderingContext2D;  
  protected _maskFilename = 'privacy-mask.svg';
  protected _drawing: boolean = false;
  protected _polygons: Point[][] = [];
  protected _offset: number = 0;
  protected _editMode: boolean = false;

  constructor(protected renderer: Renderer2) {

  }

  ngOnInit(): void {
    this.EditMode$
    .pipe(
      takeUntil(this._ngUnsubscribe$)
    )
    .subscribe((editMode: boolean) => {
      this._editMode = editMode;
    });        
  }

  ngAfterViewInit(): void {
    this._maskCanvasCtx = this.maskCanvas.nativeElement.getContext('2d');
    this.initCanvas();
  }

  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }

  getMaskAsSVG(invertMask: boolean = false) : string {
    let svgContent = `<svg xmlns="http://www.w3.org/2000/svg" width="${this.maskCanvas.nativeElement.width}" height="${this.maskCanvas.nativeElement.height}">`;
    if (invertMask) {
        // Add a black background rectangle
        svgContent += `<rect width="100%" height="100%" fill="black" />`;
        this._polygons.forEach(polygon => {
            if (polygon.length > 2) {
                const pathData = polygon.map((point, index) => {
                    const command = index === 0 ? 'M' : 'L';
                    return `${command}${point.x},${point.y}`;
                }).join(' ') + 'Z';
                svgContent += `<path d="${pathData}" fill="white" stroke="white" />`;
            }
        });
    } else {
        // Add a white background rectangle
        svgContent += `<rect width="100%" height="100%" fill="white" />`;
        this._polygons.forEach(polygon => {
            if (polygon.length > 2) {
                const pathData = polygon.map((point, index) => {
                    const command = index === 0 ? 'M' : 'L';
                    return `${command}${point.x},${point.y}`;
                }).join(' ') + 'Z';
                svgContent += `<path d="${pathData}" fill="black" stroke="black" />`;
            }
        });
    }
    svgContent += '</svg>';

    return svgContent;
  } 

  redrawCanvas() {
    this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    this._polygons.forEach((polygon, index) => {
        const isCurrentPolygon = index === this._polygons.length - 1;
        this.drawPolygon(polygon, this._editMode && isCurrentPolygon ? 'rgba(0, 0, 0, 0.1)' : 'rgba(0, 0, 0, 1)', this._editMode && isCurrentPolygon);
    });
  }

  clearMask() {
    this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    this._polygons = [];
    this._polygons.push([]);
  }

  cancel(): void {
    this.redrawCanvas();
  }

  handleResize(resize: ResizeObserverEntry) {
    if (this.haveDimensionsChanged()) {
      this.ConvasDimentionsChanged.emit();
      this.updateSize('onResize');
    }
  }

  protected drawPolygon(polygon, fillStyle, drawBorder) {
    if (polygon.length === 0 || !polygon[0]) return;
    this._maskCanvasCtx.beginPath();
    this._maskCanvasCtx.moveTo(polygon[0].x, polygon[0].y);
    for (let i = 1; i < polygon.length; i++) {this._maskCanvasCtx.lineTo(polygon[i].x, polygon[i].y);}
    this._maskCanvasCtx.closePath();
    this._maskCanvasCtx.fillStyle = fillStyle;
    this._maskCanvasCtx.fill();
    if (drawBorder) {
      this._maskCanvasCtx.setLineDash([5, 5]);
      this._maskCanvasCtx.lineDashOffset = -this._offset;
      this._maskCanvasCtx.strokeStyle = 'white';
      this._maskCanvasCtx.lineWidth = 1;
      this._maskCanvasCtx.stroke();
      this._maskCanvasCtx.setLineDash([]);
      this._maskCanvasCtx.lineDashOffset = 0;
    }
  }

  protected updateCanvasDimensions() {
    this.maskCanvas.nativeElement.width = this.streamContainer.nativeElement.clientWidth;
    this.maskCanvas.nativeElement.height = this.streamContainer.nativeElement.clientHeight;
  }

  protected haveDimensionsChanged(): boolean {
    const height = this.imageSubscription.nativeElement.offsetHeight;    
    const width = this.imageSubscription.nativeElement.offsetWidth;
    return (this.streamContainer.nativeElement.height !== (height * devicePixelRatio) 
      || this.streamContainer.nativeElement.width !== (width * devicePixelRatio));
  }

  private initCanvas(): void {
    this.maskCanvas.nativeElement.addEventListener('mousedown', (e) => {
      if (!this._editMode || e.button === 2) return;
      if (e.detail > 1) {
        const currentPolygon = this._polygons[this._polygons.length - 1];
        if (currentPolygon.length > 2) {
            this._drawing = false;
            this._polygons.push([]);
        }
        return;
      }
      this._drawing = true;
      const rect = this.maskCanvas.nativeElement.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const currentPolygon = this._polygons[this._polygons.length - 1];
      if (currentPolygon.length > 0) {
          const firstNode = currentPolygon[0];
          const distance = Math.sqrt(Math.pow(x - firstNode.x, 2) + Math.pow(y - firstNode.y, 2));
          if (distance < 10) {
              this._drawing = false;
              this._polygons.push([]);
          } else {
            currentPolygon.push({ x, y });
          }
      } else {
          currentPolygon.push({ x, y });
      }
      this.redrawCanvas();
    });

    this.maskCanvas.nativeElement.addEventListener('mousemove', (e) => {
      if (!this._drawing || !this._editMode) return;
      const rect = this.maskCanvas.nativeElement.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const currentPolygon = this._polygons[this._polygons.length - 1];
      currentPolygon.push({ x, y });
      this.redrawCanvas();
      if (this._editMode) {
          this.drawPolygon(currentPolygon, 'rgba(0, 0, 0, 0.1)', true);
      }
      currentPolygon.pop();
    });
  }

  private updateSize(triggeredBy: string) {    
    if (!this.imageSubscription) return;
    this.updateDimensions()
  }
  
  private updateDimensions() {
    if (this.haveDimensionsChanged()) {
      let height = this.imageSubscription.nativeElement.offsetHeight;    
      let width = this.imageSubscription.nativeElement.offsetWidth;
      this.renderer.setStyle(this.streamContainer.nativeElement, 'height', `${height}px`);
      this.renderer.setStyle(this.streamContainer.nativeElement, 'width', `${width}px`);
      this.streamContainer.nativeElement.height = height * devicePixelRatio;
      this.streamContainer.nativeElement.width = width * devicePixelRatio;
      this.renderer.setStyle(this.maskCanvas.nativeElement, 'height', `${height}px`);
      this.renderer.setStyle(this.maskCanvas.nativeElement, 'width', `${width}px`);
      this.maskCanvas.nativeElement.width = this.maskCanvas.nativeElement.clientWidth * devicePixelRatio;
      this.maskCanvas.nativeElement.height = this.maskCanvas.nativeElement.clientHeight * devicePixelRatio;
      this._maskCanvasCtx.scale(devicePixelRatio, devicePixelRatio);
      this.updateCanvasDimensions();
    }
  }
}
