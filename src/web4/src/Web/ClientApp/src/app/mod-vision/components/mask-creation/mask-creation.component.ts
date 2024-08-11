import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input, Output, EventEmitter, ViewChild, ElementRef, AfterViewInit, Renderer2 } from '@angular/core';
import { Subject, Observable } from 'rxjs';

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
  @Input() EditMode: boolean = false;

  @Output() ConvasDimentionsChanged = new EventEmitter();

  @ViewChild('streamContainer', {static: false}) protected streamContainer: ElementRef;
  @ViewChild('imageSubscription', {static: false}) protected imageSubscription: ElementRef;
  @ViewChild('maskCanvas', {static: false}) protected maskCanvas: ElementRef;

  protected _maskCanvasCtx: CanvasRenderingContext2D;  
  protected _maskFilename = 'privacy-mask.svg';
  protected _drawing: boolean = false;
  protected _polygons: Point[][] = [];
  protected _offset: number = 0;

  constructor(protected renderer: Renderer2) {

  }

  ngOnInit(): void {
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
    //console.log('redrawCanvas');
    this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    this._polygons.forEach((polygon, index) => {
        const isCurrentPolygon = index === this._polygons.length - 1;
        this.drawPolygon(polygon, this.EditMode && isCurrentPolygon ? 'rgba(0, 0, 0, 0.1)' : 'rgba(0, 0, 0, 1)', this.EditMode && isCurrentPolygon);
    });
    //console.log(this.polygons);
  }

  clearMask() {
    //console.log("clearMask");
    this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    this._polygons = [];
    this._polygons.push([]);
  }

  handleResize(resize: ResizeObserverEntry) {
    //console.log('onResize', resize);
    if (this.haveDimensionsChanged()) {
      this.ConvasDimentionsChanged.emit();
      this.updateSize('onResize');
    }
  }

  protected drawPolygon(polygon, fillStyle, drawBorder) {
    //console.log("drawPolygon");
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
    //console.log(this.polygon);
  }

  protected updateCanvasDimensions() {
    //console.log('updateCanvasDimensions');
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
      //console.log('maskCanvas.mousedown');
      if (!this.EditMode || e.button === 2) return;
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
      //console.log(this.polygons.length);
      const currentPolygon = this._polygons[this._polygons.length - 1];
      //console.log(currentPolygon);
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
      //console.log('maskCanvas.mousemove');
      if (!this._drawing || !this.EditMode) return;
      const rect = this.maskCanvas.nativeElement.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const currentPolygon = this._polygons[this._polygons.length - 1];
      currentPolygon.push({ x, y });
      this.redrawCanvas();
      if (this.EditMode) {
          this.drawPolygon(currentPolygon, 'rgba(0, 0, 0, 0.1)', true);
      }
      currentPolygon.pop();
    });
  }

  private updateSize(triggeredBy: string) {    
    if (!this.imageSubscription) return;
    //const height = this.imageSubscription.nativeElement.clientHeight;
    //const width = this.imageSubscription.nativeElement.clientWidth;
    //console.log(`updateSize --> h:${height} x w:${width} triggeredBy: ${triggeredBy}`);
    this.updateDimensions()
  }
  
  private updateDimensions() {
    //private updateDimensions(height: number, width: number) {
    if (this.haveDimensionsChanged()) {
      let height = this.imageSubscription.nativeElement.offsetHeight;    
      let width = this.imageSubscription.nativeElement.offsetWidth;
      //console.log(`updateDimensions --> h:${height} x w:${width}`);
      this.renderer.setStyle(this.streamContainer.nativeElement, 'height', `${height}px`);
      this.renderer.setStyle(this.streamContainer.nativeElement, 'width', `${width}px`);
      //this.streamContainer.style.height = `${height}px`;
      //this.streamContainer.style.width = `${width}px`;
      this.streamContainer.nativeElement.height = height * devicePixelRatio;
      this.streamContainer.nativeElement.width = width * devicePixelRatio;
      this.renderer.setStyle(this.maskCanvas.nativeElement, 'height', `${height}px`);
      this.renderer.setStyle(this.maskCanvas.nativeElement, 'width', `${width}px`);
      //this.maskCanvas.style.height = `${height}px`;
      //this.maskCanvas.style.width = `${width}px`;
      this.maskCanvas.nativeElement.width = this.maskCanvas.nativeElement.clientWidth * devicePixelRatio;
      this.maskCanvas.nativeElement.height = this.maskCanvas.nativeElement.clientHeight * devicePixelRatio;
      this._maskCanvasCtx.scale(devicePixelRatio, devicePixelRatio);
      this.updateCanvasDimensions();
    }
  }
}
