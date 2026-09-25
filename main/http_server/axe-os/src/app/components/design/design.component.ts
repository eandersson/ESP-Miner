import { Component, ChangeDetectionStrategy } from '@angular/core';

@Component({
    selector: 'app-design',
    templateUrl: './design.component.html',
    changeDetection: ChangeDetectionStrategy.Eager,
    standalone: false
})
export class DesignComponent {
  constructor() { }
}
