import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input, ViewChild, ElementRef, AfterViewInit, Renderer2 } from '@angular/core';
import { Subject, Observable } from 'rxjs';

interface Point {x: number; y: number};

@Component({
  selector: 'bob-mask-creation',
  templateUrl: './mask-creation.component.html',
  styleUrls: ['./mask-creation.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MaskCreationComponent implements OnInit, OnDestroy, AfterViewInit {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();
  
  @Input() ImageStream$: Observable<string>;
  @Input() EditMode: boolean = false;

  @ViewChild('streamContainer', {static: false}) streamContainer: ElementRef;
  @ViewChild('imageSubscription', {static: false}) imageSubscription: ElementRef;
  @ViewChild('maskCanvas', {static: false}) maskCanvas: ElementRef;

  maskCanvasCtx: CanvasRenderingContext2D;
  
  maskFilename = 'privacy-mask.svg';
  drawing: boolean = false;
  polygons: Point[][] = [];
  offset: number = 0;

  constructor(private renderer: Renderer2) {

  }

  ngOnInit(): void {
  }

  ngAfterViewInit(): void {
    this.maskCanvasCtx = this.maskCanvas.nativeElement.getContext('2d');
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
        this.polygons.forEach(polygon => {
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
        this.polygons.forEach(polygon => {
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
    this.maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    this.polygons.forEach((polygon, index) => {
        const isCurrentPolygon = index === this.polygons.length - 1;
        this.drawPolygon(polygon, this.EditMode && isCurrentPolygon ? 'rgba(0, 0, 0, 0.1)' : 'rgba(0, 0, 0, 1)', this.EditMode && isCurrentPolygon);
    });
    //console.log(this.polygons);
  }

  clearMask() {
    //console.log("clearMask");
    this.maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    this.polygons = [];
    this.polygons.push([]);
  }

  handleResize(resize: ResizeObserverEntry) {
    console.log('onResize', resize);
    this.updateSize('onResize');
  }  

  private initCanvas(): void {
    this.maskCanvas.nativeElement.addEventListener('mousedown', (e) => {
      //console.log('maskCanvas.mousedown');
      if (!this.EditMode || e.button === 2) return;
      if (e.detail > 1) {
        const currentPolygon = this.polygons[this.polygons.length - 1];
        if (currentPolygon.length > 2) {
            this.drawing = false;
            this.polygons.push([]);
        }
        return;
      }
      this.drawing = true;
      const rect = this.maskCanvas.nativeElement.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      //console.log(this.polygons.length);
      const currentPolygon = this.polygons[this.polygons.length - 1];
      //console.log(currentPolygon);
      if (currentPolygon.length > 0) {
          const firstNode = currentPolygon[0];
          const distance = Math.sqrt(Math.pow(x - firstNode.x, 2) + Math.pow(y - firstNode.y, 2));
          if (distance < 10) {
              this.drawing = false;
              this.polygons.push([]);
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
      if (!this.drawing || !this.EditMode) return;
      const rect = this.maskCanvas.nativeElement.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const currentPolygon = this.polygons[this.polygons.length - 1];
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
    const height = this.imageSubscription.nativeElement.clientHeight;
    const width = this.imageSubscription.nativeElement.clientWidth;
    //console.log(`updateSize --> h:${height} x w:${width} triggeredBy: ${triggeredBy}`);
    this.updateDimensions()
  }
  
  private updateDimensions() {
    //private updateDimensions(height: number, width: number) {
    let height = this.imageSubscription.nativeElement.offsetHeight;
    let width = this.imageSubscription.nativeElement.offsetWidth;
    console.log(`updateDimensions --> h:${height} x w:${width}`);
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
    this.maskCanvasCtx.scale(devicePixelRatio, devicePixelRatio);
    this.updateCanvasDimensions();
  }

  private updateCanvasDimensions() {
    //console.log('updateCanvasDimensions');
    this.maskCanvas.nativeElement.width = this.streamContainer.nativeElement.clientWidth;
    this.maskCanvas.nativeElement.height = this.streamContainer.nativeElement.clientHeight;
  }

  private drawPolygon(polygon, fillStyle, drawBorder) {
    //console.log("drawPolygon");
    if (polygon.length === 0 || !polygon[0]) return;
    this.maskCanvasCtx.beginPath();
    this.maskCanvasCtx.moveTo(polygon[0].x, polygon[0].y);
    for (let i = 1; i < polygon.length; i++) {this.maskCanvasCtx.lineTo(polygon[i].x, polygon[i].y);}
    this.maskCanvasCtx.closePath();
    this.maskCanvasCtx.fillStyle = fillStyle;
    this.maskCanvasCtx.fill();
    if (drawBorder) {
      this.maskCanvasCtx.setLineDash([5, 5]);
      this.maskCanvasCtx.lineDashOffset = -this.offset;
      this.maskCanvasCtx.strokeStyle = 'white';
      this.maskCanvasCtx.lineWidth = 1;
      this.maskCanvasCtx.stroke();
      this.maskCanvasCtx.setLineDash([]);
      this.maskCanvasCtx.lineDashOffset = 0;
    }
    //console.log(this.polygon);
  }
}
