import { Component, OnInit, ChangeDetectionStrategy, Input, AfterViewInit, Renderer2 } from '@angular/core';
import { Observable } from 'rxjs';
import { takeUntil } from 'rxjs/operators';

import { MaskCreationComponent } from '..';

@Component({
  selector: 'bob-mask-creation-svg',
  templateUrl: './mask-creation-svg.component.html',
  styleUrls: ['./mask-creation-svg.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MaskCreationSvgComponent extends MaskCreationComponent implements OnInit, AfterViewInit  {

  @Input() MaskSvg$: Observable<string>;

  protected _maskSvg: string;
  protected _maskShapes = {};

  constructor(protected renderer: Renderer2) {
    super(renderer);
  }

  override ngOnInit(): void {
    //super.ngOnInit();

    this.MaskSvg$
    .pipe(
      takeUntil(this._ngUnsubscribe$)
    )
    .subscribe((maskSvg: string) => {
      this._maskSvg = maskSvg;
      if (maskSvg) {      
        this._maskShapes = this.parse(maskSvg);
      } else {
        this._maskShapes = {};
      }
      this.redrawCanvas();
    });

    this.EditMode$
    .pipe(
      takeUntil(this._ngUnsubscribe$)
    )
    .subscribe((editMode: boolean) => {
      this._editMode = editMode;
      this.redrawCanvas();
    });       
  }

  override ngAfterViewInit(): void {
    super.ngAfterViewInit();
      this.redrawCanvas();
  }

  override clearMask() {
    super.clearMask();
    if (this._maskSvg) {
      this._maskShapes = {};
      this.redrawCanvas();
    }    
  }

  override redrawCanvas() {    
    if (this.maskCanvas && this._maskCanvasCtx) {
      this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
      if (this._editMode) {
        this._polygons.forEach((polygon, index) => {
          const isCurrentPolygon = index === this._polygons.length - 1;
          this.drawPolygon(polygon, this._editMode && isCurrentPolygon ? 'rgba(0, 0, 0, 0.1)' : 'rgba(0, 0, 0, 0.6)', this._editMode && isCurrentPolygon);
          });
      } else {
        if (this.maskCanvas.nativeElement.width > 0 && this.maskCanvas.nativeElement.height > 0) {
          this.drawSVGOnCanvas(this._maskShapes, 0.8)
        }
      }
    }
  }

  override cancel(): void {
    if (this._maskSvg) {
      this._maskShapes = this.parse(this._maskSvg);
      this.redrawCanvas();
    }
  }

  override updateCanvasDimensions() {
    super.updateCanvasDimensions();

    this.redrawCanvas();
  }

  private parse(svgString: string) {

    var parser = new DOMParser();
    var doc = parser.parseFromString(svgString, "image/svg+xml");
    var svg = doc.querySelector('svg');

    var shapes = [];
    var svgDetails = {
        width: parseFloat(svg.getAttribute('width')),
        height: parseFloat(svg.getAttribute('height')),
        shapes: shapes
    };

    // Handle polygons and polylines
    doc.querySelectorAll('polygon, polyline').forEach(element => {
        shapes.push({
            type: element.tagName,
            points: this.extractPoints(element)
        });
    });

    // Handle rectangles
    doc.querySelectorAll('rect').forEach(rect => {
        var x = parseFloat(rect.getAttribute('x')) || 0;
        var y = parseFloat(rect.getAttribute('y')) || 0;
        var width = parseFloat(rect.getAttribute('width'));
        var height = parseFloat(rect.getAttribute('height'));

        shapes.push({
            type: 'rectangle',
            points: [
                { x: x, y: y },
                { x: x + width, y: y },
                { x: x + width, y: y + height },
                { x: x, y: y + height }
            ]
        });
    });

    // Handle paths with simple commands
    doc.querySelectorAll('path').forEach(path => {
        var d = path.getAttribute('d');
        shapes.push({
            type: 'path',
            points: this.parsePath(d)
        });
    });

    return svgDetails;
  }

  private extractPoints(element) {

    var points = [];
    var rawPoints = element.getAttribute("points").trim().split(/\s+|,/);
    for (var i = 0; i < rawPoints.length; i += 2) {
        points.push({ x: parseFloat(rawPoints[i]), y: parseFloat(rawPoints[i + 1]) });
    }
    return points;
  }

  // Basic parsing of path data
  private parsePath(d) {

    var commands = d.split(/(?=[LMCZAQ])/);
    var points = [];
    commands.forEach(cmd => {
        var type = cmd[0];
        var args = cmd.slice(1).trim().split(/[ ,]+/);
        switch (type) {
            case 'M': // Moveto
            case 'L': // Lineto
                points.push({ x: parseFloat(args[0]), y: parseFloat(args[1]) });
                break;
            // Add additional cases here for 'C', 'Q', etc., if needed
        }
    });
    return points;
  }

  private scaleSVG(svgDetails, newWidth, newHeight) {    

    var scaleX = newWidth / svgDetails.width;
    var scaleY = newHeight / svgDetails.height;

    // Scale each shape in the SVG
    svgDetails.shapes.forEach(shape => {
        shape.points = shape.points.map(point => ({
            x: point.x * scaleX,
            y: point.y * scaleY
        }));
    });

    // Update the SVG dimensions in the details
    svgDetails.width = newWidth;
    svgDetails.height = newHeight;

    return svgDetails;
  }

  private drawSVGOnCanvas(svgDetails, alpha) {

    if (Object.keys(svgDetails).length === 0 && svgDetails.constructor === Object) {
      this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
        return;
    }

    this.scaleSVG(svgDetails, this.maskCanvas.nativeElement.clientWidth, this.maskCanvas.nativeElement.clientHeight);

    this._maskCanvasCtx.fillStyle = 'rgba(0, 0, 0, ' + alpha + ')';
    this._maskCanvasCtx.fillRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);

    let oldGlobalCompositeOperation = this._maskCanvasCtx.globalCompositeOperation
    this._maskCanvasCtx.globalCompositeOperation = 'destination-out';
    this._maskCanvasCtx.fillStyle = 'white';
    svgDetails.shapes.forEach(shape => {
        if (shape.type === 'path') {// || shape.type === 'polygon' || shape.type === 'rectangle') {
          this._maskCanvasCtx.beginPath();
            shape.points.forEach((point, index) => {
                if (index === 0) {
                  this._maskCanvasCtx.moveTo(point.x, point.y);
                } else {
                  this._maskCanvasCtx.lineTo(point.x, point.y);
                }
            });
            this._maskCanvasCtx.closePath();
            this._maskCanvasCtx.fill();
        }
    });
    this._maskCanvasCtx.globalCompositeOperation = oldGlobalCompositeOperation;
  }
}
